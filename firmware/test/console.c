#include "simulation.c"

int main(void) {
  double v;
  uint32_t pos = parseDouble(&v, (const uint8_t *)"12.345", 0, 7);
  printf("%d %lf\n", pos, v);
  console_send_double(v);
  console_send_str("\n");

  pos = parseDouble(&v, (const uint8_t *)"-12.345", 0, 8);
  printf("%d %lf\n", pos, v);
  console_send_double(v);
  console_send_str("\n");

  pos = parseDouble(&v, (const uint8_t *)"42", 0, 2);
  printf("%d %lf\n", pos, v);
  console_send_double(v);
  console_send_str("\n");

  pos = parseDouble(&v, (const uint8_t *)".23", 0, 3);
  printf("%d %lf\n", pos, v);
  console_send_double(v);
  console_send_str("\n");

  int i;
  pos = parseI32(&i, (const uint8_t *)"-42", 0, 3);
  printf("%d %d\n", pos, i);
  console_send_int32(i);
  console_send_str("\n");

  char *in = strdup("kinematics:zero 0 0 -590 1 0 0 0\n");
  pos = console_receive((uint8_t *)in, strlen(in));
  printf("%d %lf %lf %lf @ %lf %lf %lf %lf\n", pos,
      tool.disp.x, tool.disp.y, tool.disp.z,
      tool.rot.r, tool.rot.i, tool.rot.j, tool.rot.k);
}
