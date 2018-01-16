#ifndef __IFACE_TUM_COLOR_HPP__
#define __IFACE_TUM_COLOR_HPP__

namespace am2b_iface
{
  /***********************************************************************
   *    PARAMETERS FOR TUM COLORS 
   ***********************************************************************/

  struct tum_colors
  {



    struct blue
    {
      // Hex #65BD
      const double r;
      const double g;
      const double b;
    blue():
      r(0),
        g(101.0/255),
        b(189.0/255.0)
      {
      };

    };

    struct green
    {
      // Hex #A2AD00
      const double r;
      const double g;
      const double b;
    green():
      r(162.0/255.0),
        g(173.0/255),
        b(0.0/255.0)
      {
      };
        
        };

    struct orange
    {
      // Hex #E37222
      const double r;
      const double g;
      const double b;
    orange():
      r(227.0/255.0),
        g(114.0/255),
        b(34.0/255.0)
      {
      };
        

        };

    struct elfenbein
    {
      // Hex #DAD7CB
      const double r;
      const double g;
      const double b; 

    elfenbein():
      r(218.0/255.0),
        g(215.0/255),
        b(203.0/255.0)
      {
      };
        

    };

    struct grey
    {
      // Hex #DAD7CB
      const double r;
      const double g;
      const double b; 
    grey():
      r(156.0/255.0),
        g(157.0/255),
        b(159.0/255.0)
      {
      };
        

    };
    blue blue;
    green green;
    orange orange;
    elfenbein elfenbein;
    grey grey;
  };


}
#endif//__TUM_COLOR_HPP__
