#include "cpu.h"
#include "raylib.h"

int main(void) {
  Cpu nescpu;
  const int screenWidth = 800;
  const int screenHeight = 450;

  InitWindow(screenWidth, screenHeight, "NEStor");

  SetTargetFPS(60);

  while (!WindowShouldClose()) {
    BeginDrawing();
    ClearBackground(RAYWHITE);
    nescpu.executeInstruction();

    EndDrawing();
  }

  CloseWindow();
  return 0;
}