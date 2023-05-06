//
// Copyright 2022 Jack W. Griffin
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
// http ://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissionsand
// limitations under the License.
//
#include "MathUtils.h"
#include <cmath>

bool MathUtils::epsilonEqual(double a, double b, double eps)
{
	return std::fabs(a - b) < eps;
}

double MathUtils::interpolate(double a, double b, double percent)
{
	if (percent < 0)
		percent = 0;
	else if (percent > 1)
		percent = 1;
	return a + (b - a) * percent;
}

double MathUtils::radiansToDegrees(double rads)
{
	return rads / kPI * 180.0;
}

double MathUtils::degreesToRadians(double degrees)
{
	return degrees / 180.0 * kPI;
}
