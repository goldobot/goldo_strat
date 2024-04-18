#include <string.h>
#include <unistd.h>
#include <math.h>
#include <zmq.h>

#include <fstream>
#include <iostream>

#include "astar_wrapper.hpp"


using namespace goldobot;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define MAX(a,b) ((a>b)?a:b)


/******************************************************************************/
/**  Playground  **************************************************************/
/******************************************************************************/

StratPlayground::StratPlayground()
{
  memset (m_stat_pattern, 0, sizeof(m_stat_pattern));
  memset (m_mob_pattern, 0, sizeof(m_mob_pattern));
  memset (m_playground, 0, sizeof(m_playground));
  memset (m_stat_playground, 0, sizeof(m_stat_playground));
  m_ppm_sz = 0;
  memset (m_ppm_buff, 0, sizeof(m_ppm_buff));
}

void StratPlayground::init()
{
  int x, y;

  /* patterns */
  init_pattern(m_stat_pattern, S_PATT_SZ_CM, S_OBST_R_CM, S_EXCL);
  init_pattern(m_mob_pattern, M_PATT_SZ_CM, M_OBST_R_CM, M_EXCL);

  /* playground */
  for (y=Y_MIN_CM; y<Y_MAX_CM; y++)
  {
    for (x=X_MIN_CM; x<X_MAX_CM; x++)
    {
      m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = NO_OBST;
    }
  }

  /* playground borders */
  x = X_MIN_CM;
  for (y=Y_MIN_CM; y<Y_MAX_CM; y++)
  {
    m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
    put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM);
  }

  x = X_MAX_CM-1;
  for (y=Y_MIN_CM; y<Y_MAX_CM; y++)
  {
    m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
    put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM);
  }

  y = Y_MIN_CM;
  for (x=X_MIN_CM; x<X_MAX_CM; x++)
  {
    m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
    put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM);
  }

  y = Y_MAX_CM-1;
  for (x=X_MIN_CM; x<X_MAX_CM; x++)
  {
    m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
    put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM);
  }

  /* static obstacles */
  put_stat_rect_obst(1850, 2000,  -610,  -590);
  put_stat_rect_obst(1850, 2000,   610,   590);
  put_stat_rect_obst(1700, 2000,   -10,    10);

  /* FIXME : TODO : make these zones (adversary homes) configurable */
  put_stat_rect_obst(1700, 2000,  -450,  -150);
  put_stat_rect_obst( 500, 1100,  1100,  1500);

  /* make backup */
  memcpy(m_stat_playground, m_playground, sizeof(m_playground));

  /* ppm image for debug */
  m_ppm_sz = 0;
  memset (m_ppm_buff, 0, sizeof(m_ppm_buff));
}

void StratPlayground::init_pattern(unsigned char *_patt, 
                                   int _patt_sz_cm, int _obst_r_cm, 
                                   unsigned char _code)
{
  int x, y;
  int _obst_r2_cm = _obst_r_cm*_obst_r_cm;

  for (y=0; y<_patt_sz_cm; y++)
  {
    for (x=0; x<_patt_sz_cm; x++)
    {
      if (((x-_obst_r_cm)*(x-_obst_r_cm)+(y-_obst_r_cm)*(y-_obst_r_cm))<
          _obst_r2_cm)
      {
        _patt[(y)*(_patt_sz_cm) + x] = _code;
      }
      else
      {
        _patt[(y)*(_patt_sz_cm) + x] = NO_OBST;
      }
    }
  }

}

void StratPlayground::put_pattern(int x_cm, int y_cm, unsigned char*_patt, 
                                  int _patt_sz_cm)
{
  int xx, yy;
  int xx_abs, yy_abs;
  int _patt_sz_2_cm = _patt_sz_cm/2;

  for (yy=0; yy<_patt_sz_cm; yy++)
  {
    for (xx=0; xx<_patt_sz_cm; xx++)
    {
      xx_abs = x_cm-_patt_sz_2_cm+xx;
      yy_abs = y_cm-_patt_sz_2_cm+yy;
      if ((yy_abs>Y_MIN_CM) && (yy_abs<Y_MAX_CM) && 
          (xx_abs>X_MIN_CM) && (xx_abs<X_MAX_CM))
      {
        if (m_playground[(yy_abs+Y_OFFSET_CM)*(X_SZ_CM)+(xx_abs)]==NO_OBST)
          m_playground[(yy_abs+Y_OFFSET_CM)*(X_SZ_CM)+(xx_abs)] = 
            _patt[(yy)*(_patt_sz_cm) + xx];
      }
    }
  }
}

void StratPlayground::put_stat_rect_obst(int x_min_mm, int x_max_mm,
                                         int y_min_mm, int y_max_mm)
{
  int x, y;
  int x_min_cm = x_min_mm/10;
  int y_min_cm = y_min_mm/10;
  int x_max_cm = x_max_mm/10;
  int y_max_cm = y_max_mm/10;

  if (x_max_cm<x_min_cm)
  {
    x = x_min_cm;
    x_min_cm = x_max_cm;
    x_max_cm = x;
  }

  if (y_max_cm<y_min_cm)
  {
    y = y_min_cm;
    y_min_cm = y_max_cm;
    y_max_cm = y;
  }

  for (y=y_min_cm; y<y_max_cm; y++)
  {
    for (x=x_min_cm; x<x_max_cm; x++)
    {
      m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = S_OBST;
      put_pattern(x, y, m_stat_pattern, S_PATT_SZ_CM);
    }
  }
}

void StratPlayground::put_mob_point_obst(int x_mm, int y_mm)
{
  int x = x_mm/10;
  int y = y_mm/10;

  m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x] = M_OBST;
  put_pattern(x, y, m_mob_pattern, M_PATT_SZ_CM);
}

void StratPlayground::feed_astar(AStar & _astar)
{
  int x;
  int y;

  for (y=Y_MIN_CM; y<Y_MAX_CM; y++)
  {
    for (x=X_MIN_CM; x<X_MAX_CM; x++)
    {
      unsigned char code = m_playground[(y+Y_OFFSET_CM)*(X_SZ_CM) + x];
      switch (code) {
      case NO_OBST:
      case OLD_PATH:
      case PATH_WP:
      case PATH:
        _astar.setWay(x, y+Y_OFFSET_CM, 1);
        break;
      case M_OBST:
      case M_EXCL:
        _astar.setWall(x, y+Y_OFFSET_CM);
        break;
      case S_OBST:
      case S_EXCL:
        _astar.setWall(x, y+Y_OFFSET_CM);
        break;
      }
    }
  }
}

void StratPlayground::erase_mob_obst()
{
  memcpy(m_playground, m_stat_playground, sizeof(m_playground));
}

void StratPlayground::create_playground_ppm()
{
  unsigned char *p=m_ppm_buff;
  const int dimH = 300, dimV = 200;
  int i, j;

  m_ppm_sz = 0;
  memset (m_ppm_buff, 0, sizeof(m_ppm_buff));

  int head_sz = sprintf((char *)p, "P6\n%d %d\n255\n", dimH, dimV);
  p += head_sz;
  m_ppm_sz += head_sz;

  for (j = 0; j < dimV; ++j)
  {
    for (i = 0; i < dimH; ++i)
    {
      unsigned char color[3];
      color[0] = 255;  /* red   */
      color[1] = 255;  /* green */
      color[2] = 255;  /* blue  */

      unsigned char code = m_playground[(i)*(X_SZ_CM) + j];

      switch (code) {
      case NO_OBST:
        color[0] = 255;  /* red */
        color[1] = 255;  /* green */
        color[2] = 255;  /* blue */
        break;
      case S_OBST:
      case M_OBST:
        color[0] =   0;  /* red */
        color[1] =   0;  /* green */
        color[2] =   0;  /* blue */
        break;
      case S_EXCL:
      case M_EXCL:
        color[0] = 128;  /* red */
        color[1] = 128;  /* green */
        color[2] = 128;  /* blue */
        break;
      case OLD_PATH:
        color[0] = 255;  /* red */
        color[1] = 255;  /* green */
        color[2] = 128;  /* blue */
        break;
      case PATH_WP:
        color[0] =   0;  /* red */
        color[1] =   0;  /* green */
        color[2] = 255;  /* blue */
        break;
      case PATH:
        color[0] = 255;  /* red */
        color[1] = 255;  /* green */
        color[2] =   0;  /* blue */
        break;
      }

      p[0] = color[0];
      p[1] = color[1];
      p[2] = color[2];
      p += 3;
      m_ppm_sz += 3;
    }
  }

}

void StratPlayground::dump_playground_ppm(char *ppm_fname)
{
  FILE *fp = fopen(ppm_fname, "wb"); /* b - binary mode */
  (void) fwrite(m_ppm_buff, 1, m_ppm_sz, fp);
  (void) fclose(fp);
}

void StratPlayground::send_playground_ppm()
{
  unsigned short my_message_type = 2051;
  unsigned char compress_flag = 0;

  if (m_ppm_sz==0) return;

#if 0 /* FIXME : TODO */
  CommZmq::instance().send((const char*)(&my_message_type), 2, ZMQ_SNDMORE);
  CommZmq::instance().send((const char*)(&compress_flag), 1, ZMQ_SNDMORE);
  CommZmq::instance().send((const char*)(m_ppm_buff), m_ppm_sz, 0);
#endif
}


/******************************************************************************/
/**  A* wrapper  **************************************************************/
/******************************************************************************/

AstarWrapper AstarWrapper::s_instance;

AstarWrapper& AstarWrapper::instance()
{
  return s_instance;
}

AstarWrapper::AstarWrapper()
{
}

int AstarWrapper::init()
{
  m_path_find_pg.init();

  m_core_astar.setMatrix(m_path_find_pg.X_SZ_CM, m_path_find_pg.Y_SZ_CM);

  return 0;
}

void AstarWrapper::dbg_astar_test(int x_start_mm, int y_start_mm,
                                  int x_end_mm, int y_end_mm,
                                  int xo1_mm, int yo1_mm,
                                  int xo2_mm, int yo2_mm,
                                  int xo3_mm, int yo3_mm,
                                  char *dump_fname)
{
  int x_start = x_start_mm/10;
  int y_start = y_start_mm/10;
  int x_end   = x_end_mm/10;
  int y_end   = y_end_mm/10;
  int x_wp = 0;
  int y_wp = 0;
  bool isNewPath = false;
  int Y_OFF_CM = m_path_find_pg.Y_OFFSET_CM;
  int X_SZ_CM = m_path_find_pg.X_SZ_CM;

  /* clear playground */
  m_path_find_pg.erase_mob_obst();

  /* put mobile obstacles */
  m_path_find_pg.put_mob_point_obst(xo1_mm, yo1_mm);
  m_path_find_pg.put_mob_point_obst(xo2_mm, yo2_mm);
  m_path_find_pg.put_mob_point_obst(xo3_mm, yo3_mm);

  /* astar test */
  m_core_astar.setMatrix(m_path_find_pg.X_SZ_CM, m_path_find_pg.Y_SZ_CM);
  m_path_find_pg.feed_astar(m_core_astar);

  m_core_astar.setWay(x_start, y_start+Y_OFF_CM, 1);
  m_core_astar.setWay(x_end,   y_end+Y_OFF_CM,   1);

  m_core_astar.setStart(x_start, y_start+Y_OFF_CM);
  m_core_astar.setEnd(x_end, y_end+Y_OFF_CM);

  list<pair<UINT, UINT>> path= m_core_astar.getPathOnlyIfNeed(true, &isNewPath);
  printf ("path(OnlyIfNeed).size() = %d\n",(int)path.size());

#if 1 /* FIXME : DEBUG */
  path = m_core_astar.getPath(AStarPathType::raw);
  printf ("path(raw).size() = %d\n",(int)path.size());
  if(path.size() > 0)
  {
    list<pair<UINT, UINT>>::iterator pathIt;
    for (pathIt = path.begin(); pathIt != path.end(); pathIt++)
    {
      x_wp = pathIt->first;
      y_wp = pathIt->second;
      y_wp -= Y_OFF_CM;
      m_path_find_pg.m_playground[(y_wp+Y_OFF_CM)*(X_SZ_CM) + x_wp] = 
        m_path_find_pg.PATH;
    }
  }
#endif

  path = m_core_astar.getPath(AStarPathType::smooth);
  printf ("path(smooth).size() = %d\n",(int)path.size());
  if(path.size() > 0)
  {
    list<pair<UINT, UINT>>::iterator pathIt;
    for (pathIt = path.begin(); pathIt != path.end(); pathIt++)
    {
      x_wp = pathIt->first;
      y_wp = pathIt->second;
      y_wp -= Y_OFF_CM;
      m_path_find_pg.m_playground[(y_wp+Y_OFF_CM)*(X_SZ_CM) + x_wp] = 
        m_path_find_pg.PATH_WP;
      printf ("<%d,%d>\n",x_wp,y_wp);
    }
  }

  m_path_find_pg.m_playground[(y_start+Y_OFF_CM)*(X_SZ_CM) + x_start] = 
    m_path_find_pg.PATH_WP;
  m_path_find_pg.m_playground[(y_end+Y_OFF_CM)*(X_SZ_CM) + x_end] = 
    m_path_find_pg.PATH_WP;

  /* dump result */
  m_path_find_pg.create_playground_ppm();
  m_path_find_pg.dump_playground_ppm(dump_fname);

  printf ("\n");
}



