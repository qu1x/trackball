// Copyright © 2021 Rouven Spreckels <rs@qu1x.dev>
// SPDX-License-Identifier: BSD-2-Clause-Patent

#include "trackball.h"

// 2D vector of your linear algebra library of choice.
typedef struct {
	double x, y;
} vec2;

// 4D vector of your linear algebra library of choice.
typedef struct {
	double x, y, z, w;
} vec4;

// Caches normalization of previous cursor/finger position.
vec4 old = { .w = 0.0 };

// Maximum cursor/finger position as screen's width and height.
vec2 max = { 800, 600 };

// Usually, a cursor position event with left mouse button being pressed.
void
handle_left_button_displacement(int x, int y) {
	vec4 rot;
	// Generic selection between `float*, `double*` and `long double*`
	// implementations is controlled by the type of the output parameter `rot`.
	trackball_orbit(
		(double*)&rot,
		(double*)&old,
		(double*)&((vec2) { x, y }), // Parenthesis required as this is a macro.
		(double*)&max
	);
	// Post-multiply rotation to total camera alignment.
}

// Event when left mouse button is released again.
void
handle_left_button_release(void) {
	// Can also or instead be invoked on `handle_left_button_press()`.
	old = (vec4) { .w = 0.0 };
}

// Simulates mouse events.
int
main(void) {
	handle_left_button_displacement(400, 300);
	handle_left_button_displacement(401, 301);
	handle_left_button_release();

	handle_left_button_displacement(401, 301);
	handle_left_button_displacement(400, 300);
	handle_left_button_release();

	return 0;
}
