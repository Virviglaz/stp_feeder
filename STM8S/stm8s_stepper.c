/*
 * This file is provided under a MIT license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * MIT License
 *
 * Copyright (c) 2019 Pavel Nadein
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * STM8S open source driver
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */
#include "stm8s_stepper.h"
#include "stm8s_clk.h"

static volatile enum stepper_st status;

static u16 do_step(u16 start, u16 steps)
{
	static u32 c;
	static u16 i, half, n;
	if (start) {
		c = start;
		i = steps;
		n = 0;
		half = steps / 2;
		return 0;
	}

	n++;

	if (n < half) /* Acceleration */
		c = c - 2 * c / (4 * (n + 1));
	else	/* Deacceleration */
		c = c + 2 * c / (4 * (i - n + 2) + 1);

	return n < i ? (u16)c : 0;
}

void stepper_init(enum stepper_ch ch, u16 psc, u16 duty, u16 start, u16 steps)
{
	CLK->PCKENR1 |= CLK_PCKENR1_TIM1;

	switch (ch) {
	case STP_CH1:
		/* PC6 */
		OPT->OPT2 = 1;
		GPIOC->DDR |= 1 << 6;
		GPIOC->CR1 |= 1 << 6;
		GPIOC->CR2 |= 1 << 6;
		TIM1->CCMR1 = (7 << 4) | TIM1_CCMR_OCxPE;
		TIM1->CCER1 = TIM1_CCER1_CC1E | TIM1_CCER1_CC1P;
		TIM1->CCR1H = (u8)(duty >> 8);
		TIM1->CCR1L = (u8)duty;
		break;
	case STP_CH2:
		/* PC5 */
		OPT->OPT2 = 1;
		GPIOC->DDR |= 1 << 5;
		GPIOC->CR1 |= 1 << 5;
		GPIOC->CR2 |= 1 << 5;
		TIM1->CCMR2 = (7 << 4) | TIM1_CCMR_OCxPE;
		TIM1->CCER2 = TIM1_CCER2_CC3E | TIM1_CCER2_CC3P;
		TIM1->CCR2H = (u8)(duty >> 8);
		TIM1->CCR2L = (u8)duty;
		break;
	case STP_CH3:
		/* NOT USED IN TSOP20 */
		break;
	case STP_CH4:
		/* PC4 */
		GPIOC->DDR |= 1 << 4;
		GPIOC->CR1 |= 1 << 4;
		GPIOC->CR2 |= 1 << 4;
		TIM1->CCMR4 = (7 << 4) | TIM1_CCMR_OCxPE;
		TIM1->CCER2 = TIM1_CCER2_CC4E | TIM1_CCER2_CC4P;
		TIM1->CCR4H = (u8)(duty >> 8);
		TIM1->CCR4L = (u8)duty;
		break;
	}

	TIM1->RCR = 0;

	TIM1->PSCRH = (u8)(psc >> 8);
	TIM1->PSCRL = (u8)psc;

	TIM1->ARRH = (u8)(start >> 8);
	TIM1->ARRL = (u8)start;

	do_step(start, steps);
	status = STP_RUNNING;

	TIM1->BKR = TIM1_BKR_MOE;
	TIM1->IER = TIM1_IER_UIE;
	TIM1->EGR = TIM1_EGR_UG;
	TIM1->CR1 = TIM1_CR1_CEN;
	rim();
}

INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11)
{
	u16 next_step = do_step(0, 0);
	TIM1->SR1 = 0;

	if (next_step) {
		TIM1->ARRH = (u8)(next_step >> 8);
		TIM1->ARRL = (u8)next_step;
	} else {
		TIM1->IER = 0;
		TIM1->CR1 = 0;
		status = STP_IDLE;
	}
}

enum stepper_st stepper_status(void)
{
	return status;
}
