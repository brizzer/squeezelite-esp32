// Copyright 2018-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. 

// #include "dsps_dotprod_f32_m_ae32.S"

// This is bi quad filter form II for ESP32 processor.
	.text
	.align  4
	.global equalizer_biquad_f32
	.type   equalizer_biquad_f32,@function
// The function implements the following C code:
//esp_err_t dsps_biquad_f32_ae32(const float* input, float* output, int len, float* coef, float* w)
//  {
//    for (int i=0 ; i< len ; i++)
//    {
//        float d0 = input[i] - coef[3]*w[0] - coef[4]*w[1]; (input[i] - a[1]*w[0] - a[2]*w[1];)
//        output[i] = coef[0]*d0 +  coef[1]*w[0] + coef[2]*w[1];
//        w[1] = w[0];
//        w[0] = d0;
//    }
//    return ESP_OK;
//  }

// void filter_biquad_i16(float *val, float *coef, float *w) 
// {
//         biquad_d0 = *val - coef[3] * w[0] - coef[4] * w[1];
// 		   *val = (ISAMPLE_T)(coef[0] * biquad_d0 + coef[1] * w[0] + coef[2] * w[1]);
//         w[1] = w[0];
//         w[0] = biquad_d0;
// }

equalizer_biquad_f32: 
// val    - a2
// coeffs  - a3
// w- a4
#define val a2
#define coef a3
#define ws a4

// f0 = b0, coef[0]
// f1 = b1, coef[1]
// f2 = b2, coef[2]
// f3 = a1, coef[3]
// f4 = a2, coef[4]
// f5 = -a1
// f6 = -a2

// f7 = w0
// f8 = w1

// f9 = val

	entry	a1, 16
	// Array increment for floating point data should be 4
	lsi   f0, a3, 0 // f0 = b0, coef[0]
	lsi   f1, a3, 4 // f1 = b1, coef[1]
	lsi   f2, a3, 8 // f2 = b2, coef[2]
	lsi   f3, a3, 12 // f3 = a1, coef[3]
	lsi   f4, a3, 16 // f4 = a2, coef[4]

	
	neg.s  f5, f3   // -a[1]
	neg.s  f6, f4   // -a[2]

	lsi   f7, a4, 0 // w[0]
	lsi   f8, a4, 4 // w[1]
	
	lsi     f9, a2, 0        // f9 = val
    madd.s  f9, f7, f5   // f9 += -a1*w0, f9 = val -coef[3]*w0
    // addi    a3, a3, 4    // out++;
    mul.s   f10, f1, f7  // f10 = b1*w0, 
    madd.s  f9, f8, f6   // f9 += -a2*w1, f9 = val - coef[3]*w0 - coef[4]*w1 = biquad_d0
    madd.s  f10, f9, f0  // f10 += b0*d0, f10 = coef[0] * biquad_d0 + coef[1]*w0
    // addi    a2, a2, 4    // in++;
    madd.s  f10, f2, f8  // f10+= b2*w1, f10 - result
    mov.s   f8, f7       // w1 = w0
    mov.s   f7, f9       // w0 = d0
    // lsi     f9, a2,  0   // f9 = x[i]
    ssi	    f10, a2, 0   // val = result

    ssi     f7, a4, 0
    ssi     f8, a4, 4

	//movi.n	a2, 0 // return status ESP_OK
	retw
