from pygdbmi.gdbcontroller import GdbController
import json

gdb_path = r"/usr/bin/gdb-multiarch"

sections = {
    'sram': [0x20000000, 64 * 1024],
    'ccmram': [0x10000000, 16 * 1024],
    'nvic': [0xe000e004, 1051]
}


class NucleoDumper:
    def __init__(self):
        self.gdbmi = GdbController([gdb_path, '--interpreter=mi3'])
        self.gdbmi.write('-target-select extended-remote localhost:3333')

    def dump(self, path):
        path = str(path)
        open(path + '.sram.bin', 'wb').write(self.read_memory('sram'))
        open(path + '.ccmram.bin', 'wb').write(self.read_memory('ccmram'))
        open(path + '.nvic.bin', 'wb').write(self.read_memory('nvic'))

        meta_json = {}
        resp = self.gdbmi.write('-data-list-register-names')
        regs = []
        for elt in resp[0]['payload']['register-names']:
            regs.append({'name': elt})
        resp = self.gdbmi.write('-data-list-register-values d')
        for elt in resp[0]['payload']['register-values']:
            regs[int(elt['number'])]['value'] = int(elt['value'])
        meta_json['registers'] = regs
        json.dump(meta_json, open(path + '.json', 'w'))

    def close(self):
        # self.gdbmi.write('-target-detach')
        self.gdbmi.exit()

    def read_memory(self, section):
        chunk_size = 4092
        start, size = sections[section]
        offset = start
        end = start + size
        buff = b''
        while offset < end:
            read_size = min(end - offset, chunk_size)
            resp = self.gdbmi.write('-data-read-memory-bytes 0x{:x} {}'.format(offset, read_size))
            offset += read_size
            mem = resp[0]['payload']['memory']
            buff += bytes.fromhex(mem[0]['contents'])
        return buff


dumper = NucleoDumper()
dumper.dump('dump')
dumper.close()
