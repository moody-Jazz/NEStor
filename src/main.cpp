#include "cpu.h"
#include "raylib.h"

int main(void) {
  std::unique_ptr<NesMemory>mem = std::make_unique<NesMemory>();
  Cpu nescpu(std::move(mem));
  
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