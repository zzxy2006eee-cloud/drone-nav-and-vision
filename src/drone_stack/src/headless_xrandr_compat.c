/* Ogre 1.9 indexes an empty RandR video-mode list on monitor-less Xorg.
 * Scope this fallback to explicitly opted-in simulation child processes. */
#define _GNU_SOURCE
#include <X11/Xlib.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <string.h>
Bool XQueryExtension(Display *display, const char *name,
                     int *opcode, int *event, int *error) {
  typedef Bool (*Query)(Display *,const char *,int *,int *,int *);
  Query query=(Query)dlsym(RTLD_NEXT,"XQueryExtension");
  const char *enabled=getenv("DRONE_HEADLESS_RANDR_WORKAROUND");
  if(enabled && strcmp(enabled,"1")==0 && name && strcmp(name,"RANDR")==0) {
    if(opcode)*opcode=0;if(event)*event=0;if(error)*error=0;
    return False;
  }
  return query ? query(display,name,opcode,event,error) : False;
}
