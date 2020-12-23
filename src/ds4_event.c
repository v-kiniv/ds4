#include "ds4_internal.h"

void ds4_gen_event(struct mgos_ds4_input_arg *event, 
                     struct mgos_ds4_state *prev, struct mgos_ds4_state *next)
{
  int i;
  struct mgos_ds4_button *bd = &event->button_down;
  struct mgos_ds4_button *bu = &event->button_up;

  // Button down events
  bd->options   = !prev->button.options  && next->button.options;
  bd->l3        = !prev->button.l3       && next->button.l3;
  bd->r3        = !prev->button.r3       && next->button.r3;
  bd->share     = !prev->button.share    && next->button.share;

  bd->up        = !prev->button.up       && next->button.up;
  bd->right     = !prev->button.right    && next->button.right;
  bd->down      = !prev->button.down     && next->button.down;
  bd->left      = !prev->button.left     && next->button.left;

  bd->upright   = !prev->button.upright  && next->button.upright;
  bd->upleft    = !prev->button.upleft   && next->button.upleft;
  bd->downright = !prev->button.downright&& next->button.downright;
  bd->downleft  = !prev->button.downleft && next->button.downleft;

  bd->l2        = !prev->button.l2       && next->button.l2;
  bd->r2        = !prev->button.r2       && next->button.r2;
  bd->l1        = !prev->button.l1       && next->button.l1;
  bd->r1        = !prev->button.r1       && next->button.r1;

  bd->triangle  = !prev->button.triangle && next->button.triangle;
  bd->circle    = !prev->button.circle   && next->button.circle;
  bd->cross     = !prev->button.cross    && next->button.cross;
  bd->square    = !prev->button.square   && next->button.square;

  bd->ps        = !prev->button.ps       && next->button.ps;
  bd->touchpad  = !prev->button.touchpad && next->button.touchpad;

  // Button up events
  bu->options   = prev->button.options  && !next->button.options;
  bu->l3        = prev->button.l3       && !next->button.l3;
  bu->r3        = prev->button.r3       && !next->button.r3;
  bu->share     = prev->button.share    && !next->button.share;

  bu->up        = prev->button.up       && !next->button.up;
  bu->right     = prev->button.right    && !next->button.right;
  bu->down      = prev->button.down     && !next->button.down;
  bu->left      = prev->button.left     && !next->button.left;

  bu->upright   = prev->button.upright  && !next->button.upright;
  bu->upleft    = prev->button.upleft   && !next->button.upleft;
  bu->downright = prev->button.downright&& !next->button.downright;
  bu->downleft  = prev->button.downleft && !next->button.downleft;

  bu->l2        = prev->button.l2       && !next->button.l2;
  bu->r2        = prev->button.r2       && !next->button.r2;
  bu->l1        = prev->button.l1       && !next->button.l1;
  bu->r1        = prev->button.r1       && !next->button.r1;

  bu->triangle  = prev->button.triangle && !next->button.triangle;
  bu->circle    = prev->button.circle   && !next->button.circle;
  bu->cross     = prev->button.cross    && !next->button.cross;
  bu->square    = prev->button.square   && !next->button.square;

  bu->ps        = prev->button.ps       && !next->button.ps;
  bu->touchpad  = prev->button.touchpad && !next->button.touchpad;

  // Sticks
  for(i = 0; i < 2; i++) {
    event->stick_move[i] = prev->stick[i].x != next->stick[i].x ||
                           prev->stick[i].y != next->stick[i].y ||
                           prev->stick[i].z != next->stick[i].z;
  }

  // Triggers
  for(i = 0; i < 2; i++) {
    event->trigger_move[i] = prev->trigger[i] != next->trigger[i];
  }

  // Gyroscope
  event->gyro_move = prev->gyro.x != next->gyro.x ||
                     prev->gyro.y != next->gyro.y ||
                     prev->gyro.z != next->gyro.z;

  // Accelerometer
  event->accel_move = prev->accel.x != next->accel.x ||
                      prev->accel.y != next->accel.y ||
                      prev->accel.z != next->accel.z;
}
