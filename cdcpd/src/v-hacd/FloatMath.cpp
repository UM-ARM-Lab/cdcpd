#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include "v-hacd/FloatMath.h"
#include <vector>

#define REAL float

#include "v-hacd/FloatMath.inl"

#undef REAL
#define REAL double

#include "v-hacd/FloatMath.inl"