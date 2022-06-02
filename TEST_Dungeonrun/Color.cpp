#include "framework.h"
Color::Color() {
	c = (u32*)malloc(sizeof(u32));
	rad = (Color::radiance*)malloc(sizeof(Color::radiance));
};
Color::Color(u64 size) {
	c = (u32*)malloc(sizeof(u32) * size);
	rad = (Color::radiance*)malloc(sizeof(Color::radiance) * size);
};