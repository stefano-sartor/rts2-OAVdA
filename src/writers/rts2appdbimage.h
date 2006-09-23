#ifndef __RTS2_APP_DB_IMAGE__
#define __RTS2_APP_DB_IMAGE__

#include "../utilsdb/rts2appdb.h"
#include "rts2imagedb.h"

#include <list>

class Rts2AppDbImage:public Rts2AppDb
{
protected:
  std::list < const char *>imageNames;
  virtual int processImage (Rts2ImageDb * image)
  {
    return 0;
  }
public:
    Rts2AppDbImage (int in_argc, char **in_argv):Rts2AppDb (in_argc, in_argv)
  {
  }
  virtual ~ Rts2AppDbImage (void)
  {
    imageNames.clear ();
  }
  virtual int processArgs (const char *in_arg)
  {
    imageNames.push_back (in_arg);
    return 0;
  }

  virtual int run ()
  {
    int ret = 0;
    std::list < const char *>::iterator img_iter;
    for (img_iter = imageNames.begin (); img_iter != imageNames.end ();
	 img_iter++)
      {
	const char *an_name = *img_iter;
	Rts2ImageDb *imagedb = new Rts2ImageDb (an_name);
	Rts2ImageDb *image = getValueImageType (imagedb);
	ret = processImage (image);
	delete image;
	if (ret)
	  return ret;
      }
    return ret;
  }
};

#endif /* !__RTS2_APP_DB_IMAGE__ */
