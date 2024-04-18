#pragma once
#include <cstdint>
#include <cstddef>

#include "astar.hpp"

namespace goldobot
{

/**  Playground  *************************************************************/

  class StratPlayground {
  public:
    StratPlayground();

    void init();

    void init_pattern(unsigned char *_patt, 
                      int _patt_sz_cm, int _obst_r_cm, 
                      unsigned char _code);

    void put_pattern(int x_cm, int y_cm, unsigned char*_patt, int _patt_sz_cm);

    void put_stat_rect_obst(int x_min_mm, int x_max_mm,
                            int y_min_mm, int y_max_mm);

    void put_mob_point_obst(int x_mm, int y_mm);

    void erase_mob_obst();

    void feed_astar(AStar & _astar);

    void create_playground_ppm();

    void dump_playground_ppm(char *ppm_fname);

    void send_playground_ppm();


    static const int Y_OFFSET_CM     = 150;
    static const int X_MIN_CM        =   0;
    static const int X_MAX_CM        = 200;
    static const int X_SZ_CM         = X_MAX_CM-X_MIN_CM;
    static const int Y_MIN_CM        =-150;
    static const int Y_MAX_CM        = 150;
    static const int Y_SZ_CM         = Y_MAX_CM-Y_MIN_CM;
    static const int S_OBST_R_CM     =  14;
    static const int M_OBST_R_CM     =  34;
    static const int S_PATT_SZ_CM    = 2*S_OBST_R_CM;
    static const int M_PATT_SZ_CM    = 2*M_OBST_R_CM;

    static const unsigned char NO_OBST  = 255;
    static const unsigned char S_OBST   =   0;
    static const unsigned char M_OBST   =   1;
    static const unsigned char S_EXCL   =  64;
    static const unsigned char M_EXCL   =  65;
    static const unsigned char PATH     = 128;
    static const unsigned char OLD_PATH = 129;
    static const unsigned char PATH_WP  = 160;

    unsigned char m_playground[X_SZ_CM * Y_SZ_CM];
    unsigned char m_stat_playground[X_SZ_CM * Y_SZ_CM];
    unsigned char m_stat_pattern[S_PATT_SZ_CM * S_PATT_SZ_CM];
    unsigned char m_mob_pattern[M_PATT_SZ_CM * M_PATT_SZ_CM];

    int m_ppm_sz;
    unsigned char m_ppm_buff[0x40000];
  };


/**  A* wrapper  *************************************************************/

  class AstarWrapper
  {
  public:
    static AstarWrapper& instance();

    AstarWrapper();

    int init();

    void dbg_astar_test(int x_start_mm, int y_start_mm,
                        int x_end_mm, int y_end_mm,
                        int xo1_mm, int yo1_mm,
                        int xo2_mm, int yo2_mm,
                        int xo3_mm, int yo3_mm,
                        char *dump_fname);

  private:

    StratPlayground m_path_find_pg;

    AStar m_core_astar;

    static AstarWrapper s_instance;
  };


}

