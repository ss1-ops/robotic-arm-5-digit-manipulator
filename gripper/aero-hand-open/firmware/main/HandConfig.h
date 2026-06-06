// HandConfig.h
#pragma once

// Choose one via build flags OR by uncommenting a line below:
//   - PlatformIO:   build_flags = -DRIGHT_HAND   (or -DLEFT_HAND)
//   - Arduino CLI:  --build-property compiler.cpp.extra_flags="-DRIGHT_HAND"
//   - Arduino IDE:  just uncomment one of the lines here.

//#define LEFT_HAND
#define RIGHT_HAND

#if !defined(LEFT_HAND) && !defined(RIGHT_HAND)
  #error "Define exactly one: LEFT_HAND or RIGHT_HAND (build flag or uncomment in HandConfig.h)."
#endif

#if defined(LEFT_HAND) && defined(RIGHT_HAND)
  #error "Do not define both LEFT_HAND and RIGHT_HAND."
#endif