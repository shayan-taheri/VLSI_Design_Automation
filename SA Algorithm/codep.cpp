/*
 * codep.cpp
 *
 *  Created on: Oct 19, 2013
 *      Author: shayantaheri
 */

#include <iostream>
#include <limits>
#include <algorithm>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>
using namespace std;

/* Defining the Cost function. It is needed to consider 4 inputs for this function. Also, regarding the fact that
   the designed function needs 3 functions in its structure (according to my design) and it is out of the main function,
   I defined a class with 3 functions as its elements to make a nesting call function. */

float cost(char exx[39], char op[20], float area[20], float ratio[20])
{

// Defining the class with 3 needed functions !

	class local
	{
		public:

		// Function "maxn" gives the maximum of two float numbers.

		static float maxn (float a, float b) {
			if (a > b)
				return a;
			else
				return b;
		}

		// Function "minn" gives the minimum of two float numbers.

		static float minn (float c, float d) {
			if (c < d)
				return c;
			else
				return d;
		}

		/* Function "check" is for checking procedure of the results of two leaves (Children). It means that this functions will check
		   the overlapping state of four achieved results. In 'H' operation, a1, a2, a3, and a4 are for heights and b1, b2, b3, and b4 are
		   for widths and vice versa. */

		static void check (float& a1, float& a2, float& b1, float& b2) {
				float mog1 = 0.0;
				float mog2 = 0.0;
				if ((a1 == a2) && (b2 < b1)) {
						b1 = b2; }
				if ((a1 < a2) || (a2 < a1)) {
					mog1 = a1 * b1;
					mog2 = a2 * b2;
					if (mog2 < mog1) {
						a1 = a2;
						b1 = b2;}}

				}

	};

	// Start point for defining the local variables of the cost function.

	/* In this code, I have designed and coded the cost function with the following algorithm.
	   We can consider every two operands and an operator from the left of the NPE as a subset (subtree) of
	   normalized polish expression and replace it with a new operand such as 'x'.
	   So, if we have an operator ('H' or 'V') in location 'i' then our operands should be in
	   locations 'i-1' and 'i-2'. */

	/* After each operation ('H' or 'V'), we would have four results and we can keep two of them according
	   to the overlapping ("check" function) and replace the two other ones with the future new results.
	   But, in the last operation, there would be four results and even after the "check" function,
	   we should have 4 available spaces for them. So, we should consider 40 = (2 * 19) + 2 spaces for
	   new height and width arrays: "h_comp" and "w_comp" */

	char ex1[39];
	char op2[19];
	float h[20];
	float w[20];
	float h_comp[76];
	float w_comp[76];
	float temp1_h[8];
	float temp1_w[8];
	float temp2_h[16];
	float temp2_w[16];
	float rt1 = 0.0;
	float rt2 = 0.0;
	float rt3 = 0.0;
	float rt4 = 0.0;
	float res1 = 0.0;
	float res2 = 0.0;
	float result = 0.0;
	float h1 = 0.0;
	float h2 = 0.0;
	float w1 = 0.0;
	float w2 = 0.0;
	int c = 0;
	int p = 38;
	int j = 0;
	int k = 0;
	char ex2[39];
	int con1 = 0;
	int con2 = 0;
	int con3 = 0;
	int con4 = 0;
	char val1, val2, val3, val4;
	int i = 0;
	int ars = 0;
	int jj = 0;
	int kk = 0;
	int mot1 = 0;
	int mot2 = 0;

	// Obtaining height and width of modules according to their area (= h * w) and ratio (= h / w).

	for (int q = 0; q <= 5; q++) {
		w[q] = sqrt (area[q] / ratio [q]);
		h[q] = area[q] / w[q];}

	for (int rfv = 6; rfv <= 19; rfv++) {
		w[rfv] = 0;
		h[rfv] = 0;}

	for (int yo = 0; yo <= 38; yo++) {
		ex1[yo] = 0;
		ex2[yo] = 0;}

	for (int fcd = 0; fcd <= 10; fcd++) {
		ex1[fcd] = exx[fcd];}

	for (int eo = 0; eo <= 75; eo++) {
		h_comp[eo] = 0;
		w_comp[eo] = 0;}

	for (int cxs = 0; cxs <= 7; cxs++) {
		temp1_h[cxs] = 0;
		temp1_w[cxs] = 0;}

	for (int cxs = 0; cxs <= 15; cxs++) {
		temp2_h[cxs] = 0;
		temp2_w[cxs] = 0;}

	for (int ghj = 0; ghj <= 18; ghj++) {
		op2[ghj] = 0;}

	/* Start point for checking each element of NPE for finding an operator ('H' or 'V') and then applying
	   the explained algorithm. */

	for (; i <= 38; i++) {

		// If the element 'i' of the NPE is equal to 'V'.

		if ((ex1[i] == 'V') && (c <= 72)) {

			for (; j <= 19; j++) {
			if ((ex1[i-1] == op[j])) {
				val1 = op[j];
				h1 = h[j];
				w1 = w[j];
				j = 20;
			}}

			for (; k <= 19; k++) {
			if ((ex1[i-2] == op[k])) {
				val2 = op[k];
				h2 = h[k];
				w2 = w[k];
				k = 20;
			}}

			for (; jj <= 18; jj++) {
						if ((ex1[i-1] == op2[jj])) {
							val3 = op2[jj];
							jj = 19;}}

							if ( val3 == 'A') {
							mot1 = 0;}
							if ( val3 == 'B') {
							mot1 = 4;}
							if ( val3 == 'C') {
							mot1 = 8;}
							if ( val3 == 'D') {
							mot1 = 12;}
							if ( val3 == 'E') {
							mot1 = 16;}
							if ( val3 == 'F') {
							mot1 = 20;}
							if ( val3 == 'G') {
							mot1 = 24;}
							if ( val3 == 'I') {
							mot1 = 28;}
							if ( val3 == 'J') {
							mot1 = 32;}
							if ( val3 == 'K') {
							mot1 = 36;}
							if ( val3 == 'L') {
							mot1 = 40;}
							if ( val3 == 'M') {
							mot1 = 44;}
							if ( val3 == 'N') {
							mot1 = 48;}
							if ( val3 == 'O') {
							mot1 = 52;}
							if ( val3 == 'P') {
							mot1 = 56;}
							if ( val3 == 'Q') {
							mot1 = 60;}
							if ( val3 == 'R') {
							mot1 = 64;}
							if ( val3 == 'S') {
							mot1 = 68;}
							if ( val3 == 'T') {
							mot1 = 72;}

							for (; kk <= 18; kk++) {
										if ((ex1[i-2] == op2[kk])) {
											val4 = op2[kk];
											kk = 19;}}

											if ( val4 == 'A') {
											mot2 = 0;}
											if ( val4 == 'B') {
											mot2 = 4;}
											if ( val4 == 'C') {
											mot2 = 8;}
											if ( val4 == 'D') {
											mot2 = 12;}
											if ( val4 == 'E') {
											mot2 = 16;}
											if ( val4 == 'F') {
											mot2 = 20;}
											if ( val4 == 'G') {
											mot2 = 24;}
											if ( val4 == 'I') {
											mot2 = 28;}
											if ( val4 == 'J') {
											mot2 = 32;}
											if ( val4 == 'K') {
											mot2 = 36;}
											if ( val4 == 'L') {
											mot2 = 40;}
											if ( val4 == 'M') {
											mot2 = 44;}
											if ( val4 == 'N') {
											mot2 = 48;}
											if ( val4 == 'O') {
											mot2 = 52;}
											if ( val4 == 'P') {
											mot2 = 56;}
											if ( val4 == 'Q') {
											mot2 = 60;}
											if ( val4 == 'R') {
											mot2 = 64;}
											if ( val4 == 'S') {
											mot2 = 68;}
											if ( val4 == 'T') {
											mot2 = 72;}

		// If both of the elements 'i-1' and 'i-2' are in the operands.

		if ((ex1[i-1] == val1) && (ex1[i-2] == val2)) {
			w_comp[c] = h1 + h2;
			w_comp[c+1] = w1 + w2;
			w_comp[c+2] = h1 + w2;
			w_comp[c+3] = w1 + h2;
			h_comp[c] = local::maxn (w1, w2);
			h_comp[c+1] = local::maxn (h1, h2);
			h_comp[c+2] = local::maxn (w1, h2);
			h_comp[c+3] = local::maxn (h1, w2);

			con1 = 1;}

		// If the element 'i-2' is in expression and element 'i-1' is a new operand (= 'x').

		if ((ex1[i-2] == val2) && (ex1[i-1] == val3)) {
						temp1_w[0] = w2 + w_comp[mot1];
						temp1_w[1] = w2 + w_comp[mot1+1];
						temp1_w[2] = w2 + w_comp[mot1+2];
						temp1_w[3] = w2 + w_comp[mot1+3];

						temp1_w[4] = h2 + w_comp[mot1];
						temp1_w[5] = h2 + w_comp[mot1+1];
						temp1_w[6] = h2 + w_comp[mot1+2];
						temp1_w[7] = h2 + w_comp[mot1+3];

						temp1_h[0] = local::maxn (h2, h_comp[mot1]);
						temp1_h[1] = local::maxn (h2, h_comp[mot1+1]);
						temp1_h[2] = local::maxn (h2, h_comp[mot1+2]);
						temp1_h[3] = local::maxn (h2, h_comp[mot1+3]);

						temp1_h[4] = local::maxn (w2, h_comp[mot1]);
						temp1_h[5] = local::maxn (w2, h_comp[mot1+1]);
						temp1_h[6] = local::maxn (w2, h_comp[mot1+2]);
						temp1_h[7] = local::maxn (w2, h_comp[mot1+3]);

						local::check (temp1_h[0], temp1_h[4], temp1_w[0], temp1_w[4]);
						local::check (temp1_h[1], temp1_h[5], temp1_w[1], temp1_w[5]);
						local::check (temp1_h[2], temp1_h[6], temp1_w[2], temp1_w[6]);
						local::check (temp1_h[3], temp1_h[7], temp1_w[3], temp1_w[7]);

						h_comp[c] = temp1_h[0];
						h_comp[c+1] = temp1_h[1];
						h_comp[c+2] = temp1_h[2];
						h_comp[c+3] = temp1_h[3];

						w_comp[c] = temp1_w[0];
						w_comp[c+1] = temp1_w[1];
						w_comp[c+2] = temp1_w[2];
						w_comp[c+3] = temp1_w[3];
			con2 = 1;}

		// If the element 'i-1' is in expression and element 'i-2' is a new operand (= 'x').

		if ((ex1[i-1] == val1) && (ex1[i-2] == val4)) {
						temp1_w[0] = w1 + w_comp[mot2];
						temp1_w[1] = w1 + w_comp[mot2+1];
						temp1_w[2] = w1 + w_comp[mot2+2];
						temp1_w[3] = w1 + w_comp[mot2+3];

						temp1_w[4] = h1 + w_comp[mot2];
						temp1_w[5] = h1 + w_comp[mot2+1];
						temp1_w[6] = h1 + w_comp[mot2+2];
						temp1_w[7] = h1 + w_comp[mot2+3];

						temp1_h[0] = local::maxn (h1, h_comp[mot2]);
						temp1_h[1] = local::maxn (h1, h_comp[mot2+1]);
						temp1_h[2] = local::maxn (h1, h_comp[mot2+2]);
						temp1_h[3] = local::maxn (h1, h_comp[mot2+3]);

						temp1_h[4] = local::maxn (w1, h_comp[mot2]);
						temp1_h[5] = local::maxn (w1, h_comp[mot2+1]);
						temp1_h[6] = local::maxn (w1, h_comp[mot2+2]);
						temp1_h[7] = local::maxn (w1, h_comp[mot2+3]);

						local::check (temp1_h[0], temp1_h[4], temp1_w[0], temp1_w[4]);
						local::check (temp1_h[1], temp1_h[5], temp1_w[1], temp1_w[5]);
						local::check (temp1_h[2], temp1_h[6], temp1_w[2], temp1_w[6]);
						local::check (temp1_h[3], temp1_h[7], temp1_w[3], temp1_w[7]);

						h_comp[c] = temp1_h[0];
						h_comp[c+1] = temp1_h[1];
						h_comp[c+2] = temp1_h[2];
						h_comp[c+3] = temp1_h[3];

						w_comp[c] = temp1_w[0];
						w_comp[c+1] = temp1_w[1];
						w_comp[c+2] = temp1_w[2];
						w_comp[c+3] = temp1_w[3];
			con3 = 1;}

		// If both of the elements 'i-1' and 'i-2' are new operands (each of them = 'x').

		if ((ex1[i-1] == val3) && (ex1[i-2] == val4)) {
						temp2_w[0] = w_comp[mot1] + w_comp[mot2];
						temp2_w[1] = w_comp[mot1] + w_comp[mot2+1];
						temp2_w[2] = w_comp[mot1] + w_comp[mot2+2];
						temp2_w[3] = w_comp[mot1] + w_comp[mot2+3];

						temp2_w[4] = w_comp[mot1+1] + w_comp[mot2];
						temp2_w[5] = w_comp[mot1+1] + w_comp[mot2+1];
						temp2_w[6] = w_comp[mot1+1] + w_comp[mot2+2];
						temp2_w[7] = w_comp[mot1+1] + w_comp[mot2+3];

						temp2_w[8] = w_comp[mot1+2] + w_comp[mot2];
						temp2_w[9] = w_comp[mot1+2] + w_comp[mot2+1];
						temp2_w[10] = w_comp[mot1+2] + w_comp[mot2+2];
						temp2_w[11] = w_comp[mot1+2] + w_comp[mot2+3];

						temp2_w[12] = w_comp[mot1+3] + w_comp[mot2];
						temp2_w[13] = w_comp[mot1+3] + w_comp[mot2+1];
						temp2_w[14] = w_comp[mot1+3] + w_comp[mot2+2];
						temp2_w[15] = w_comp[mot1+3] + w_comp[mot2+3];

						temp2_h[0] = local::maxn (h_comp[mot1], h_comp[mot2]);
						temp2_h[1] = local::maxn (h_comp[mot1], h_comp[mot2+1]);
						temp2_h[2] = local::maxn (h_comp[mot1], h_comp[mot2+2]);
						temp2_h[3] = local::maxn (h_comp[mot1], h_comp[mot2+3]);

						temp2_h[4] = local::maxn (h_comp[mot1+1], h_comp[mot2]);
						temp2_h[5] = local::maxn (h_comp[mot1+1], h_comp[mot2+1]);
						temp2_h[6] = local::maxn (h_comp[mot1+1], h_comp[mot2+2]);
						temp2_h[7] = local::maxn (h_comp[mot1+1], h_comp[mot2+3]);

						temp2_h[8] = local::maxn (h_comp[mot1+2], h_comp[mot2]);
						temp2_h[9] = local::maxn (h_comp[mot1+2], h_comp[mot2+1]);
						temp2_h[10] = local::maxn (h_comp[mot1+2], h_comp[mot2+2]);
						temp2_h[11] = local::maxn (h_comp[mot1+2], h_comp[mot2+3]);

						temp2_h[12] = local::maxn (h_comp[mot1+3], h_comp[mot2]);
						temp2_h[13] = local::maxn (h_comp[mot1+3], h_comp[mot2+1]);
						temp2_h[14] = local::maxn (h_comp[mot1+3], h_comp[mot2+2]);
						temp2_h[15] = local::maxn (h_comp[mot1+3], h_comp[mot2+3]);

						local::check (temp2_h[0], temp2_h[4], temp2_w[0], temp2_w[4]);
						local::check (temp2_h[1], temp2_h[5], temp2_w[1], temp2_w[5]);
						local::check (temp2_h[2], temp2_h[6], temp2_w[2], temp2_w[6]);
						local::check (temp2_h[3], temp2_h[7], temp2_w[3], temp2_w[7]);

						local::check (temp2_h[8], temp2_h[12], temp2_w[8], temp2_w[12]);
						local::check (temp2_h[9], temp2_h[13], temp2_w[9], temp2_w[13]);
						local::check (temp2_h[10], temp2_h[14], temp2_w[10], temp2_w[14]);
						local::check (temp2_h[11], temp2_h[15], temp2_w[11], temp2_w[15]);

						local::check (temp2_h[0], temp2_h[8], temp2_w[0], temp2_w[8]);
						local::check (temp2_h[1], temp2_h[9], temp2_w[1], temp2_w[9]);
						local::check (temp2_h[2], temp2_h[10], temp2_w[2], temp2_w[10]);
						local::check (temp2_h[3], temp2_h[11], temp2_w[3], temp2_w[11]);

						h_comp[c] = temp2_h[0];
						h_comp[c+1] = temp2_h[1];
						h_comp[c+2] = temp2_h[2];
						h_comp[c+3] = temp2_h[3];

						w_comp[c] = temp2_w[0];
						w_comp[c+1] = temp2_w[1];
						w_comp[c+2] = temp2_w[2];
						w_comp[c+3] = temp2_w[3];
			con4 = 1;}

		/* We can assign a 'z' character or any other character (except the initial operands and 'x') to
		   the last two cells of the expression each time to don't consider them in the next iteration again ! */

		// Reconstructing the new normalized polish expression according to the new situation :

		if ((con1 == 1) || (con2 == 1) || (con3 == 1) || (con4 == 1))
		{

		if (p > 1) {
			if (c == 0) {
						ex1[i-2] = 'A';
						op2[ars] = 'A';}
						if (c == 4) {
						ex1[i-2] = 'B';
						op2[ars] = 'B';}
						if (c == 8) {
						ex1[i-2] = 'C';
						op2[ars] = 'C';}
						if (c == 12) {
						ex1[i-2] = 'D';
						op2[ars] = 'D';}
						if (c == 16) {
						ex1[i-2] = 'E';
						op2[ars] = 'E';}
						if (c == 20) {
						ex1[i-2] = 'F';
						op2[ars] = 'F';}
						if (c == 24) {
						ex1[i-2] = 'G';
						op2[ars] = 'G';}
						if (c == 28) {
						ex1[i-2] = 'I';
						op2[ars] = 'I';}
						if (c == 32) {
						ex1[i-2] = 'J';
						op2[ars] = 'J';}
						if (c == 36) {
						ex1[i-2] = 'K';
						op2[ars] = 'K';}
						if (c == 40) {
						ex1[i-2] = 'L';
						op2[ars] = 'L';}
						if (c == 44) {
						ex1[i-2] = 'M';
						op2[ars] = 'M';}
						if (c == 48) {
						ex1[i-2] = 'N';
						op2[ars] = 'N';}
						if (c == 52) {
						ex1[i-2] = 'O';
						op2[ars] = 'O';}
						if (c == 56) {
						ex1[i-2] = 'P';
						op2[ars] = 'P';}
						if (c == 60) {
						ex1[i-2] = 'Q';
						op2[ars] = 'Q';}
						if (c == 64) {
						ex1[i-2] = 'R';
						op2[ars] = 'R';}
						if (c == 68) {
						ex1[i-2] = 'S';
						op2[ars] = 'S';}
						if (c == 72) {
						ex1[i-2] = 'T';
						op2[ars] = 'T';}
		for (int s = 0; s <= 37-i; s++) {
		ex2[s] = ex1[i+1+s]; }
		for (int v = 0; v <= 37-i; v++) {
		ex1[i-1+v] = ex2[v];}
		ex1[p] = 'z';
		ex1[p-1] = 'z';
		p = p - 2;
		i = 0;
		j = 0;
		k = 0;
		jj = 0;
		kk = 0;
		c = c + 4;
	ars = ars + 1;
		con1 = 0;
		con2 = 0;
		con3 = 0;
		con4 = 0;}
		if (p < 1) {
		i = 39;}

		}

		}

		// The following procedure (ex1[i] = 'H') is approximately the same as the previous procedure (ex1[i] = 'V') :

		if ((ex1[i] == 'H') && (c <= 72)) {

			for (; j <= 19; j++) {
						if ((ex1[i-1] == op[j])) {
							val1 = op[j];
							h1 = h[j];
							w1 = w[j];
							j = 20;
						}}

						for (; k <= 19; k++) {
						if ((ex1[i-2] == op[k])) {
							val2 = op[k];
							h2 = h[k];
							w2 = w[k];
							k = 20;
						}}

						for (; jj <= 18; jj++) {
									if ((ex1[i-1] == op2[jj])) {
										val3 = op2[jj];
										jj = 19;}}

						if ( val3 == 'A') {
													mot1 = 0;}
													if ( val3 == 'B') {
													mot1 = 4;}
													if ( val3 == 'C') {
													mot1 = 8;}
													if ( val3 == 'D') {
													mot1 = 12;}
													if ( val3 == 'E') {
													mot1 = 16;}
													if ( val3 == 'F') {
													mot1 = 20;}
													if ( val3 == 'G') {
													mot1 = 24;}
													if ( val3 == 'I') {
													mot1 = 28;}
													if ( val3 == 'J') {
													mot1 = 32;}
													if ( val3 == 'K') {
													mot1 = 36;}
													if ( val3 == 'L') {
													mot1 = 40;}
													if ( val3 == 'M') {
													mot1 = 44;}
													if ( val3 == 'N') {
													mot1 = 48;}
													if ( val3 == 'O') {
													mot1 = 52;}
													if ( val3 == 'P') {
													mot1 = 56;}
													if ( val3 == 'Q') {
													mot1 = 60;}
													if ( val3 == 'R') {
													mot1 = 64;}
													if ( val3 == 'S') {
													mot1 = 68;}
													if ( val3 == 'T') {
													mot1 = 72;}

										for (; kk <= 18; kk++) {
													if ((ex1[i-2] == op2[kk])) {
														val4 = op2[kk];
														kk = 19;}}

										if ( val4 == 'A') {
																					mot2 = 0;}
																					if ( val4 == 'B') {
																					mot2 = 4;}
																					if ( val4 == 'C') {
																					mot2 = 8;}
																					if ( val4 == 'D') {
																					mot2 = 12;}
																					if ( val4 == 'E') {
																					mot2 = 16;}
																					if ( val4 == 'F') {
																					mot2 = 20;}
																					if ( val4 == 'G') {
																					mot2 = 24;}
																					if ( val4 == 'I') {
																					mot2 = 28;}
																					if ( val4 == 'J') {
																					mot2 = 32;}
																					if ( val4 == 'K') {
																					mot2 = 36;}
																					if ( val4 == 'L') {
																					mot2 = 40;}
																					if ( val4 == 'M') {
																					mot2 = 44;}
																					if ( val4 == 'N') {
																					mot2 = 48;}
																					if ( val4 == 'O') {
																					mot2 = 52;}
																					if ( val4 == 'P') {
																					mot2 = 56;}
																					if ( val4 == 'Q') {
																					mot2 = 60;}
																					if ( val4 == 'R') {
																					mot2 = 64;}
																					if ( val4 == 'S') {
																					mot2 = 68;}
																					if ( val4 == 'T') {
																					mot2 = 72;}

		if ((ex1[i-1] == val1) && (ex1[i-2] == val2)) {
			h_comp[c] = h1 + h2;
			h_comp[c+1] = w1 + w2;
			h_comp[c+2] = h1 + w2;
			h_comp[c+3] = w1 + h2;
			w_comp[c] = local::maxn (w1, w2);
			w_comp[c+1] = local::maxn (h1, h2);
			w_comp[c+2] = local::maxn (w1, h2);
			w_comp[c+3] = local::maxn (h1, w2);

			con1 = 1;}

		if ((ex1[i-2] == val2) && (ex1[i-1] == val3)) {
						temp1_h[0] = h2 + h_comp[mot1];
						temp1_h[1] = h2 + h_comp[mot1+1];
						temp1_h[2] = h2 + h_comp[mot1+2];
						temp1_h[3] = h2 + h_comp[mot1+3];

						temp1_h[4] = w2 + h_comp[mot1];
						temp1_h[5] = w2 + h_comp[mot1+1];
						temp1_h[6] = w2 + h_comp[mot1+2];
						temp1_h[7] = w2 + h_comp[mot1+3];

						temp1_w[0] = local::maxn (w2, w_comp[mot1]);
						temp1_w[1] = local::maxn (w2, w_comp[mot1+1]);
						temp1_w[2] = local::maxn (w2, w_comp[mot1+2]);
						temp1_w[3] = local::maxn (w2, w_comp[mot1+3]);

						temp1_w[4] = local::maxn (h2, w_comp[mot1]);
						temp1_w[5] = local::maxn (h2, w_comp[mot1+1]);
						temp1_w[6] = local::maxn (h2, w_comp[mot1+2]);
						temp1_w[7] = local::maxn (h2, w_comp[mot1+3]);

						local::check (temp1_w[0], temp1_w[4], temp1_h[0], temp1_h[4]);
						local::check (temp1_w[1], temp1_w[5], temp1_h[1], temp1_h[5]);
						local::check (temp1_w[2], temp1_w[6], temp1_h[2], temp1_h[6]);
						local::check (temp1_w[3], temp1_w[7], temp1_h[3], temp1_h[7]);

						h_comp[c] = temp1_h[0];
						h_comp[c+1] = temp1_h[1];
						h_comp[c+2] = temp1_h[2];
						h_comp[c+3] = temp1_h[3];

						w_comp[c] = temp1_w[0];
						w_comp[c+1] = temp1_w[1];
						w_comp[c+2] = temp1_w[2];
						w_comp[c+3] = temp1_w[3];
			con2 = 1;}

		if ((ex1[i-1] == val1) && (ex1[i-2] == val4)) {
						temp1_h[0] = h1 + h_comp[mot2];
						temp1_h[1] = h1 + h_comp[mot2+1];
						temp1_h[2] = h1 + h_comp[mot2+2];
						temp1_h[3] = h1 + h_comp[mot2+3];

						temp1_h[4] = w1 + h_comp[mot2];
						temp1_h[5] = w1 + h_comp[mot2+1];
						temp1_h[6] = w1 + h_comp[mot2+2];
						temp1_h[7] = w1 + h_comp[mot2+3];

						temp1_w[0] = local::maxn (w1, w_comp[mot2]);
						temp1_w[1] = local::maxn (w1, w_comp[mot2+1]);
						temp1_w[2] = local::maxn (w1, w_comp[mot2+2]);
						temp1_w[3] = local::maxn (w1, w_comp[mot2+3]);

						temp1_w[4] = local::maxn (h1, w_comp[mot2]);
						temp1_w[5] = local::maxn (h1, w_comp[mot2+1]);
						temp1_w[6] = local::maxn (h1, w_comp[mot2+2]);
						temp1_w[7] = local::maxn (h1, w_comp[mot2+3]);

						local::check (temp1_w[0], temp1_w[4], temp1_h[0], temp1_h[4]);
						local::check (temp1_w[1], temp1_w[5], temp1_h[1], temp1_h[5]);
						local::check (temp1_w[2], temp1_w[6], temp1_h[2], temp1_h[6]);
						local::check (temp1_w[3], temp1_w[7], temp1_h[3], temp1_h[7]);

						h_comp[c] = temp1_h[0];
						h_comp[c+1] = temp1_h[1];
						h_comp[c+2] = temp1_h[2];
						h_comp[c+3] = temp1_h[3];

						w_comp[c] = temp1_w[0];
						w_comp[c+1] = temp1_w[1];
						w_comp[c+2] = temp1_w[2];
						w_comp[c+3] = temp1_w[3];
			con3 = 1;}

		if ((ex1[i-1] == val3) && (ex1[i-2] == val4)) {
						temp2_h[0] = h_comp[mot1] + h_comp[mot2];
						temp2_h[1] = h_comp[mot1] + h_comp[mot2+1];
						temp2_h[2] = h_comp[mot1] + h_comp[mot2+2];
						temp2_h[3] = h_comp[mot1] + h_comp[mot2+3];

						temp2_h[4] = h_comp[mot1+1] + h_comp[mot2];
						temp2_h[5] = h_comp[mot1+1] + h_comp[mot2+1];
						temp2_h[6] = h_comp[mot1+1] + h_comp[mot2+2];
						temp2_h[7] = h_comp[mot1+1] + h_comp[mot2+3];

						temp2_h[8] = h_comp[mot1+2] + h_comp[mot2];
						temp2_h[9] = h_comp[mot1+2] + h_comp[mot2+1];
						temp2_h[10] = h_comp[mot1+2] + h_comp[mot2+2];
						temp2_h[11] = h_comp[mot1+2] + h_comp[mot2+3];

						temp2_h[12] = h_comp[mot1+3] + h_comp[mot2];
						temp2_h[13] = h_comp[mot1+3] + h_comp[mot2+1];
						temp2_h[14] = h_comp[mot1+3] + h_comp[mot2+2];
						temp2_h[15] = h_comp[mot1+3] + h_comp[mot2+3];

						temp2_w[0] = local::maxn (w_comp[mot1], w_comp[mot2]);
						temp2_w[1] = local::maxn (w_comp[mot1], w_comp[mot2+1]);
						temp2_w[2] = local::maxn (w_comp[mot1], w_comp[mot2+2]);
						temp2_w[3] = local::maxn (w_comp[mot1], w_comp[mot2+3]);

						temp2_w[4] = local::maxn (w_comp[mot1+1], w_comp[mot2]);
						temp2_w[5] = local::maxn (w_comp[mot1+1], w_comp[mot2+1]);
						temp2_w[6] = local::maxn (w_comp[mot1+1], w_comp[mot2+2]);
						temp2_w[7] = local::maxn (w_comp[mot1+1], w_comp[mot2+3]);

						temp2_w[8] = local::maxn (w_comp[mot1+2], w_comp[mot2]);
						temp2_w[9] = local::maxn (w_comp[mot1+2], w_comp[mot2+1]);
						temp2_w[10] = local::maxn (w_comp[mot1+2], w_comp[mot2+2]);
						temp2_w[11] = local::maxn (w_comp[mot1+2], w_comp[mot2+3]);

						temp2_w[12] = local::maxn (w_comp[mot1+3], w_comp[mot2]);
						temp2_w[13] = local::maxn (w_comp[mot1+3], w_comp[mot2+1]);
						temp2_w[14] = local::maxn (w_comp[mot1+3], w_comp[mot2+2]);
						temp2_w[15] = local::maxn (w_comp[mot1+3], w_comp[mot2+3]);

						local::check (temp2_w[0], temp2_w[4], temp2_h[0], temp2_h[4]);
						local::check (temp2_w[1], temp2_w[5], temp2_h[1], temp2_h[5]);
						local::check (temp2_w[2], temp2_w[6], temp2_h[2], temp2_h[6]);
						local::check (temp2_w[3], temp2_w[7], temp2_h[3], temp2_h[7]);

						local::check (temp2_w[8], temp2_w[12], temp2_h[8], temp2_h[12]);
						local::check (temp2_w[9], temp2_w[13], temp2_h[9], temp2_h[13]);
						local::check (temp2_w[10], temp2_w[14], temp2_h[10], temp2_h[14]);
						local::check (temp2_w[11], temp2_w[15], temp2_h[11], temp2_h[15]);

						local::check (temp2_w[0], temp2_w[8], temp2_h[0], temp2_h[8]);
						local::check (temp2_w[1], temp2_w[9], temp2_h[1], temp2_h[9]);
						local::check (temp2_w[2], temp2_w[10], temp2_h[2], temp2_h[10]);
						local::check (temp2_w[3], temp2_w[11], temp2_h[3], temp2_h[11]);

						h_comp[c] = temp2_h[0];
						h_comp[c+1] = temp2_h[1];
						h_comp[c+2] = temp2_h[2];
						h_comp[c+3] = temp2_h[3];

						w_comp[c] = temp2_w[0];
						w_comp[c+1] = temp2_w[1];
						w_comp[c+2] = temp2_w[2];
						w_comp[c+3] = temp2_w[3];
			con4 = 1;}

		if ((con1 == 1) || (con2 == 1) || (con3 == 1) || (con4 == 1))
		{

			if (p > 1) {
				if (c == 0) {
										ex1[i-2] = 'A';
										op2[ars] = 'A';}
										if (c == 4) {
										ex1[i-2] = 'B';
										op2[ars] = 'B';}
										if (c == 8) {
										ex1[i-2] = 'C';
										op2[ars] = 'C';}
										if (c == 12) {
										ex1[i-2] = 'D';
										op2[ars] = 'D';}
										if (c == 16) {
										ex1[i-2] = 'E';
										op2[ars] = 'E';}
										if (c == 20) {
										ex1[i-2] = 'F';
										op2[ars] = 'F';}
										if (c == 24) {
										ex1[i-2] = 'G';
										op2[ars] = 'G';}
										if (c == 28) {
										ex1[i-2] = 'I';
										op2[ars] = 'I';}
										if (c == 32) {
										ex1[i-2] = 'J';
										op2[ars] = 'J';}
										if (c == 36) {
										ex1[i-2] = 'K';
										op2[ars] = 'K';}
										if (c == 40) {
										ex1[i-2] = 'L';
										op2[ars] = 'L';}
										if (c == 44) {
										ex1[i-2] = 'M';
										op2[ars] = 'M';}
										if (c == 48) {
										ex1[i-2] = 'N';
										op2[ars] = 'N';}
										if (c == 52) {
										ex1[i-2] = 'O';
										op2[ars] = 'O';}
										if (c == 56) {
										ex1[i-2] = 'P';
										op2[ars] = 'P';}
										if (c == 60) {
										ex1[i-2] = 'Q';
										op2[ars] = 'Q';}
										if (c == 64) {
										ex1[i-2] = 'R';
										op2[ars] = 'R';}
										if (c == 68) {
										ex1[i-2] = 'S';
										op2[ars] = 'S';}
										if (c == 72) {
										ex1[i-2] = 'T';
										op2[ars] = 'T';}
						for (int s = 0; s <= 37-i; s++) {
						ex2[s] = ex1[i+1+s]; }
						for (int v = 0; v <= 37-i; v++) {
						ex1[i-1+v] = ex2[v];}
						ex1[p] = 'z';
						ex1[p-1] = 'z';
						p = p - 2;
						i = 0;
						j = 0;
						k = 0;
						jj = 0;
						kk = 0;
						c = c + 4;
						ars = ars + 1;
						con1 = 0;
						con2 = 0;
						con3 = 0;
						con4 = 0;}
						if (p < 1) {
						i = 39;}

		}

		}

	}

	// Checking the minimum size of the area for final result of the cost function:

	rt1 = h_comp[16] * w_comp[16];
	rt2 = h_comp[17] * w_comp[17];
	res1 = local::minn (rt1, rt2);

	rt3 = h_comp[18] * w_comp[18];
	rt4 = h_comp[19] * w_comp[19];
	res2 = local::minn (rt1, rt2);

	result = local::minn (res1, res2);

	return result;

}

int main()
{

	// Defining the normalized polish expression (EXPRESSION) and the local variables of "main" function.

	// *** PLEASE SET "EXPRESSION[39]" TO ONE OF THE Normalized Polish Expression Topologies : ***

	// According to the tutorial of the assignment, We have six modules --> 6 Operands and 5 Operators

	float AREA[20];
	float RATIO[20];
	char operands[20];
	char E0[39] = {'1','2','V','3','V','4','V','5','V','6','V'};
	float cost_result_E = 0.0;
	float cost_result_EN = 0.0;
	float cost_result_Best = 0.0;
	float cost_result_E0 = 0.0;

	/* The Normalized Polish Expression Topologies :

	   a. {'1','2','V','3','V','4','V','5','V','6','V'}
	   b. {'1','2','H','3','H','4','H','5','H','6','H'}

	*/

	// Reading the input file and assigning its content to the equivalent arrays :

	ifstream fp ("input_file.txt");
	if (fp.is_open())
	{
		while ( !fp.eof() )
		{
			for (int o = 0; o <= 19; o++) {
				fp >> operands[o];
				fp >> AREA[o];
				fp >> RATIO[o];}
		}
	}

	for (int qaz = 11; qaz <= 38; qaz++) {
		E0[qaz] = 0;}

	for (int plm = 6; plm <= 19; plm++) {
		operands[plm] = 0;
		AREA[plm] = 0;
		RATIO[plm] = 0;}

	// Specifying the parameters for completing the Simulated Annealing Algorithm

	char Best[11];
	float T0 = 0.0;

	// After running the algorithm for 60 times, I got 60 values for delta_cost. So, the average of them is:

	float delta_avg = 42.381269/60;
	float P = 0.99;
	float T = 0.0;
	int uphill = 0;
	int MT = 0;
	int Reject = 0;
	float delta_cost = 0.0;
	char vnew0 = 0;
	char vnew1 = 0;
	char vnew2 = 0;
	char vnew3 = 0;
	char vnew4 = 0;
	char vnew5 = 0;
	char vnew6 = 0;
	char swp1 = 0;
	char E[11];
	char E_new[11];
	char operators[2] = {'H', 'V'};
	int KC = 0;
	int M = 0;
	int E_i = 0;
	int E_i1 = 0;
	int E_i2 = 0;
	int E_i3 = 0;
	int E_i4 = 0;
	int E_i5 = 0;
	int E_i6 = 0;
	int E_x = 0;
	int s_con1 = 0;
	int s_con2 = 0;
	int s_con3 = 0;
	int s_con4 = 0;
	int s_con5 = 0;
	int operands_j = 0;
	int operands_k = 0;
	char U[11];
	int cx = 0;
	int crx = 0;
	int Done;
	int opr_count = 0;
	int opn_count = 0;
	int n_chance = 0;
	int bb = 0;
	int cc = 0;

	for (int gff = 0; gff <= 10; gff++) {
	E[gff] = E0[gff];
	Best[gff] = E0[gff];
	E_new[gff] = E0[gff];
	U[gff] = 0;}
	T0 = -(delta_avg / log(P));
	T = T0;

	// Defining the first loop in SA algorithm which has condition for "Temperature"

	do {

		MT = 0;
		uphill = 0;
		Reject = 0;

		// Defining the second loop in SA algorithm which is about the selecting one of the paths of swapping !

		do {

		if (uphill <= 60) {
		M = 1 + (rand() % 3);
		switch (M) {

		// Swapping two adjacent operands:

		case 1 :

			s_con1 = 0;
			s_con2 = 0;
			s_con3 = 0;
			s_con4 = 0;
			s_con5 = 0;
			E_i1 = 0;
			E_i2 = 0;
			E_i3 = 0;
			E_i4 = 0;
			E_i5 = 0;
			E_i6 = 0;
			operands_j = 0;
			operands_k = 0;
			n_chance = 0;
			bb = 0;
			vnew0 = 0;
			vnew1 = 0;
			vnew2 = 0;
			vnew3 = 0;
			vnew4 = 0;
			vnew5 = 0;
			vnew6 = 0;
			swp1 = 0;

			bb = E_i;
			for (; (bb <= 9) && ((bb + 3) <= 10) && ((bb - 3) >= 0); bb++) {
			for (; operands_j <= 5; operands_j++) {
			if ((E_new[bb] == operands[operands_j])) {
			vnew0 = operands[operands_j];
			operands_j = 6;
			E_i = bb;
			bb = 10;}}}

			operands_j = 0;

			E_i1 = E_i + 1;
			E_i2 = E_i + 2;
			E_i3 = E_i + 3;
			E_i4 = E_i - 1;
			E_i5 = E_i - 2;
			E_i6 = E_i - 3;

			operands_k = 0;

			for (; operands_k <= 5; operands_k++) {
			if (E_new[E_i1] == operands[operands_k]) {
			vnew1 = operands[operands_k];
			operands_k = 6;}}

			operands_k = 0;

			for (; operands_k <= 5; operands_k++) {
			if (E_new[E_i2] == operands[operands_k]) {
			vnew2 = operands[operands_k];
			operands_k = 6;}}

			operands_k = 0;

			for (; operands_k <= 5; operands_k++) {
			if (E_new[E_i3] == operands[operands_k]) {
			vnew3 = operands[operands_k];
			operands_k = 6;}}

			operands_k = 0;

			for (; operands_k <= 5; operands_k++) {
			if (E_new[E_i4] == operands[operands_k]) {
			vnew4 = operands[operands_k];
			operands_k = 6;}}

			operands_k = 0;

			for (; operands_k <= 5; operands_k++) {
			if (E_new[E_i5] == operands[operands_k]) {
			vnew5 = operands[operands_k];
			operands_k = 6;}}

			operands_k = 0;

			for (; operands_k <= 5; operands_k++) {
			if (E_new[E_i6] == operands[operands_k]) {
			vnew6 = operands[operands_k];
			operands_k = 6;}}

			operands_k = 0;

			if ((E_new[E_i] == vnew0) && (E_new[E_i1] == vnew1)) {
			swp1 = E_new[E_i];
			E_new[E_i] = E_new[E_i1];
			E_new[E_i1] = swp1;
			s_con1 = 1;}

			if ((E_new[E_i] == vnew0) && (E_new[E_i2] == vnew2) && (s_con1 == 0)) {
			swp1 = E_new[E_i];
			E_new[E_i] = E_new[E_i2];
			E_new[E_i2] = swp1;
			s_con2 = 1;}

			if ((E_new[E_i] == vnew0) && (E_new[E_i3] == vnew3) && (s_con1 == 0) && (s_con2 == 0)) {
			swp1 = E_new[E_i];
			E_new[E_i] = E_new[E_i3];
			E_new[E_i3] = swp1;
			s_con3 = 1;}

			if ((E_new[E_i] == vnew0) && (E_new[E_i4] == vnew4) && (s_con1 == 0) && (s_con2 == 0) && (s_con3 == 0)) {
			swp1 = E_new[E_i];
			E_new[E_i] = E_new[E_i4];
			E_new[E_i4] = swp1;
			s_con4 = 1;}

			if ((E_new[E_i] == vnew0) && (E_new[E_i5] == vnew5) && (s_con1 == 0) && (s_con2 == 0) && (s_con3 == 0) && (s_con4 == 0)) {
			swp1 = E_new[E_i];
			E_new[E_i] = E_new[E_i5];
			E_new[E_i5] = swp1;
			s_con5 = 1;}

			if ((E_new[E_i] == vnew0) && (E_new[E_i6] == vnew6) && (s_con1 == 0) && (s_con2 == 0) && (s_con3 == 0) && (s_con4 == 0) && (s_con5 == 0)) {
			swp1 = E_new[E_i];
			E_new[E_i] = E_new[E_i6];
			E_new[E_i6] = swp1;}

			if (E_i <= 8) {
				E_i = E_i + 1;}
			if (!(E_i <= 8)) {
				E_i = 0;}

		break;

		// Complementing the operators of a random chain:

		case 2 :

			KC = 2 + (rand() % 9);
			cx = 2 + (rand() % (KC-1));
			crx = cx;
			for (; cx <= KC; cx++) {
			U[cx] = E_new[cx];}
			cx = crx;
			for (; cx <= KC; cx++) {
			if (U[cx] == 'H') {
			U[cx] = 'X';}
			if (U[cx] == 'V') {
			U[cx] = 'Y';}
			if (U[cx] == 'X') {
			U[cx] = 'V';}
			if (U[cx] == 'Y') {
			U[cx] = 'H';}}
			cx = crx;
			for (; cx <= KC; cx++) {
			E_new[cx] = U[cx];}
			cx = 0;

		break;

		// Swapping two adjacent operator and operand

		case 3 :

			Done = 0;
			while (Done == 0) {

				s_con1 = 0;
				s_con2 = 0;
				s_con3 = 0;
				s_con4 = 0;
				s_con5 = 0;
				E_i1 = 0;
				E_i2 = 0;
				E_i3 = 0;
				E_i4 = 0;
				E_i5 = 0;
				E_i6 = 0;
				E_x = 0;
				operands_j = 0;
				operands_k = 0;
				n_chance = 0;
				cc = 0;
				vnew0 = 0;
				vnew1 = 0;
				vnew2 = 0;
				vnew3 = 0;
				vnew4 = 0;
				vnew5 = 0;
				vnew6 = 0;
				swp1 = 0;
				opr_count = 0;
				opn_count = 0;

				cc = E_i;
				for (; (cc <= 10) && ((cc + 3) <= 10) && ((cc - 3) >= 0); cc++) {
				for (; operands_j <= 1; operands_j++) {
				if ((E_new[cc] == operators[operands_j])) {
				vnew0 = operators[operands_j];
				operands_j = 2;
				E_i = cc;
				cc = 10;}}}

				operands_j = 0;

			E_x = E_i;
			E_i1 = E_i + 1;
			E_i2 = E_i + 2;
			E_i3 = E_i + 3;
			E_i4 = E_i - 1;
			E_i5 = E_i - 2;
			E_i6 = E_i - 3;

			operands_k = 0;

			for (; operands_k <= 5; operands_k++) {
			if (E_new[E_i1] == operands[operands_k]) {
			vnew1 = operands[operands_k];
			operands_k = 6;}}

			operands_k = 0;

			for (; operands_k <= 5; operands_k++) {
			if (E_new[E_i2] == operands[operands_k]) {
			vnew2 = operands[operands_k];
			operands_k = 6;}}

			operands_k = 0;

			for (; operands_k <= 5; operands_k++) {
			if (E_new[E_i3] == operands[operands_k]) {
			vnew3 = operands[operands_k];
			operands_k = 6;}}

			operands_k = 0;

			for (; operands_k <= 5; operands_k++) {
			if (E_new[E_i4] == operands[operands_k]) {
			vnew4 = operands[operands_k];
			operands_k = 6;}}

			operands_k = 0;

			for (; operands_k <= 5; operands_k++) {
			if (E_new[E_i5] == operands[operands_k]) {
			vnew5 = operands[operands_k];
			operands_k = 6;}}

			operands_k = 0;

			for (; operands_k <= 5; operands_k++) {
			if (E_new[E_i6] == operands[operands_k]) {
			vnew6 = operands[operands_k];
			operands_k = 6;}}

			operands_k = 0;
			opr_count = 0;
			opn_count = 0;

			if ((E_new[E_i] == vnew0) && (E_new[E_i1] == vnew1)) {
			swp1 = E_new[E_i];
			E_new[E_i] = E_new[E_i1];
			E_new[E_i1] = swp1;
			s_con1 = 1;
			Done = 1;

			for (int E_in = 0; E_in <= E_x; E_in++) {
			for (int opc = 0; opc <= 5; opc++) {
			if (E_new[E_in] == operands[opc]) {
			opn_count = opn_count + 1;}}}

			for (int E_in = 0; E_in <= E_x; E_in++) {
			for (int opc = 0; opc <= 1; opc++) {
			if (E_new[E_in] == operators[opc]) {
			opr_count = opr_count + 1;}}}

			if (!(opr_count < opn_count)) {
			swp1 = E_new[E_i1];
			E_new[E_i1] = E_new[E_i];
			E_new[E_i] = swp1;
			s_con1 = 0;
			Done = 0;}
			}

			opr_count = 0;
			opn_count = 0;

			if ((E_new[E_i] == vnew0) && (E_new[E_i2] == vnew2) && (s_con1 == 0)) {
			swp1 = E_new[E_i];
			E_new[E_i] = E_new[E_i2];
			E_new[E_i2] = swp1;
			s_con2 = 1;
			Done = 1;

			for (int E_in = 0; E_in <= E_x; E_in++) {
			for (int opc = 0; opc <= 5; opc++) {
			if (E_new[E_in] == operands[opc]) {
			opn_count = opn_count + 1;}}}

			for (int E_in = 0; E_in <= E_x; E_in++) {
			for (int opc = 0; opc <= 1; opc++) {
			if (E_new[E_in] == operators[opc]) {
			opr_count = opr_count + 1;}}}

			if (!(opr_count < opn_count)) {
			swp1 = E_new[E_i2];
			E_new[E_i2] = E_new[E_i];
			E_new[E_i] = swp1;
			s_con2 = 0;
			Done = 0;}
			}

			opr_count = 0;
			opn_count = 0;

			if ((E_new[E_i] == vnew0) && (E_new[E_i3] == vnew3) && (s_con1 == 0) && (s_con2 == 0)) {
			swp1 = E_new[E_i];
			E_new[E_i] = E_new[E_i3];
			E_new[E_i3] = swp1;
			s_con3 = 1;
			Done = 1;

			for (int E_in = 0; E_in <= E_x; E_in++) {
			for (int opc = 0; opc <= 5; opc++) {
			if (E_new[E_in] == operands[opc]) {
			opn_count = opn_count + 1;}}}

			for (int E_in = 0; E_in <= E_x; E_in++) {
			for (int opc = 0; opc <= 1; opc++) {
			if (E_new[E_in] == operators[opc]) {
			opr_count = opr_count + 1;}}}

			if (!(opr_count < opn_count)) {
			swp1 = E_new[E_i3];
			E_new[E_i3] = E_new[E_i];
			E_new[E_i] = swp1;
			s_con3 = 0;
			Done = 0;}
			}

			opr_count = 0;
			opn_count = 0;

			if ((E_new[E_i] == vnew0) && (E_new[E_i4] == vnew4) && (s_con1 == 0) && (s_con2 == 0) && (s_con3 == 0)) {
			swp1 = E_new[E_i];
			E_new[E_i] = E_new[E_i4];
			E_new[E_i4] = swp1;
			s_con4 = 1;
			Done = 1;

			for (int E_in = 0; E_in <= E_x; E_in++) {
			for (int opc = 0; opc <= 5; opc++) {
			if (E_new[E_in] == operands[opc]) {
			opn_count = opn_count + 1;}}}

			for (int E_in = 0; E_in <= E_x; E_in++) {
			for (int opc = 0; opc <= 1; opc++) {
			if (E_new[E_in] == operators[opc]) {
			opr_count = opr_count + 1;}}}

			if (!(opr_count < opn_count)) {
			swp1 = E_new[E_i4];
			E_new[E_i4] = E_new[E_i];
			E_new[E_i] = swp1;
			s_con4 = 0;
			Done = 0;}
			}

			opr_count = 0;
			opn_count = 0;

			if ((E_new[E_i] == vnew0) && (E_new[E_i5] == vnew5) && (s_con1 == 0) && (s_con2 == 0) && (s_con3 == 0) && (s_con4 == 0)) {
			swp1 = E_new[E_i];
			E_new[E_i] = E_new[E_i5];
			E_new[E_i5] = swp1;
			s_con5 = 1;
			Done = 1;

			for (int E_in = 0; E_in <= E_x; E_in++) {
			for (int opc = 0; opc <= 5; opc++) {
			if (E_new[E_in] == operands[opc]) {
			opn_count = opn_count + 1;}}}

			for (int E_in = 0; E_in <= E_x; E_in++) {
			for (int opc = 0; opc <= 1; opc++) {
			if (E_new[E_in] == operators[opc]) {
			opr_count = opr_count + 1;}}}

			if (!(opr_count < opn_count)) {
			swp1 = E_new[E_i5];
			E_new[E_i5] = E_new[E_i];
			E_new[E_i] = swp1;
			s_con5 = 0;
			Done = 0;}
			}

			opr_count = 0;
			opn_count = 0;

			if ((E_new[E_i] == vnew0) && (E_new[E_i6] == vnew6) && (s_con1 == 0) && (s_con2 == 0) && (s_con3 == 0) && (s_con4 == 0) && (s_con5 == 0)) {
			swp1 = E_new[E_i];
			E_new[E_i] = E_new[E_i6];
			E_new[E_i6] = swp1;
			Done = 1;

			for (int E_in = 0; E_in <= E_x; E_in++) {
			for (int opc = 0; opc <= 5; opc++) {
			if (E_new[E_in] == operands[opc]) {
			opn_count = opn_count + 1;}}}

			for (int E_in = 0; E_in <= E_x; E_in++) {
			for (int opc = 0; opc <= 1; opc++) {
			if (E_new[E_in] == operators[opc]) {
			opr_count = opr_count + 1;}}}

			if (!(opr_count < opn_count)) {
			swp1 = E_new[E_i6];
			E_new[E_i6] = E_new[E_i];
			E_new[E_i] = swp1;
			Done = 0;}
			}
			if (E_i <= 9) {
				E_i = E_i + 1;}
			if (!(E_i <= 9)) {
				E_i = 0;}
			}

		break;


		default :
		break;
					}

		MT = MT + 1;

		// Accepting E_new for E or Best, if it can satisfy the conditions:

		cost_result_E = cost (E, operands, AREA, RATIO);
		cost_result_EN = cost (E_new, operands, AREA, RATIO);

		delta_cost = cost_result_EN - cost_result_E;

		if ((delta_cost < 0) || ((rand() % 2) < exp(-(delta_cost/T)))) {
			if (delta_cost > 0) {
				uphill = uphill + 1;}

			for (int poi = 0; poi <= 10; poi++) {
				E[poi] = E_new[poi];}

			cost_result_Best = cost(Best, operands, AREA, RATIO);
			cost_result_E = cost (E, operands, AREA, RATIO);

			if (cost_result_E < cost_result_Best) {
			for (int por = 0; por <= 10; por++) {
				Best[por] = E[por];}}}

		if (!((delta_cost < 0) || ((rand() % 2) < exp(-(delta_cost/T))))) {
				Reject = Reject + 1;}

		}} while (MT <= 120);

		T = 0.85 * T;

	} while ((T > 0.001) && ((Reject / MT) <= 0.95));

		// Outputting the Initial Topology (NPE).

		cout << "The Initial Topology (NPE) is: \n";

		for (int fyy = 0; fyy <= 10; fyy++) {

			cout << E0[fyy] << " ";
		}

		cost_result_E0 = cost (E0, operands, AREA, RATIO);

		// Outputting the Initial Cost of Floorplan (area).

		cout << "\n\nThe Initial Cost of Floorplan is: \n" << cost_result_E0 << "\n";



		// Outputting the Final Topology (NPE).

		cout << "\nThe Final Topology (NPE) is: \n";

		for (int fyx = 0; fyx <= 10; fyx++) {

			cout << Best[fyx] << " ";
		}

		cost_result_Best = cost (Best, operands, AREA, RATIO);

		// Outputting the Initial Cost of Floorplan (area).

		cout << "\n\nThe Final Cost of Floorplan is: \n" << cost_result_Best << "\n";

	return 0;

}
