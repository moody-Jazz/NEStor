#include "cpu.h"
#include "raylib.h"
#include "cartridge.h"

int main(void) {
  std::string filepath = "donkey kong.nes";
  std::vector<uint8_t> romBuffer{ loadRom(filepath) };
  std::unique_ptr<Mapper> mapper = createMapper(romBuffer);
  
  std::unique_ptr<NesMemory>mem = std::make_unique<NesMemory>(std::move(mapper));

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