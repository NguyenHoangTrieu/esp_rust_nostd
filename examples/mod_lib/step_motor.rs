use esp_hal::gpio::Output;
use esp_hal::gpio::Level;
use esp_hal::delay::Delay;

pub struct Stepper<'a> {
    in1: Output<'a>,
    in2: Output<'a>,
    in3: Output<'a>,
    in4: Output<'a>,
    delay: &'a mut Delay,
}

impl<'a> Stepper<'a> {
    pub fn new(
        in1: Output<'a>,
        in2: Output<'a>,
        in3: Output<'a>,
        in4: Output<'a>,
        delay: &'a mut Delay,
    ) -> Self {
        Self { in1, in2, in3, in4, delay }
    }

    const SEQUENCE: [[bool; 4]; 4] = [
        [true, false, false, true],
        [true, true, false, false],
        [false, true, true, false],
        [false, false, true, true],
    ];

    pub fn step_forward(&mut self, steps: usize, delay_ms: u32) {
        for _ in 0..steps {
            for pattern in Self::SEQUENCE.iter() {
                self.set_step(pattern);
                self.delay.delay_millis(delay_ms);
            }
        }
    }

    pub fn step_backward(&mut self, steps: usize, delay_ms: u32) {
        for _ in 0..steps {
            for pattern in Self::SEQUENCE.iter().rev() {
                self.set_step(pattern);
                self.delay.delay_millis(delay_ms);
            }
        }
    }

    fn set_step(&mut self, pattern: &[bool; 4]) {
        self.in1.set_level(Level::from(pattern[0]));
        self.in2.set_level(Level::from(pattern[1]));
        self.in3.set_level(Level::from(pattern[2]));
        self.in4.set_level(Level::from(pattern[3]));
    }
}