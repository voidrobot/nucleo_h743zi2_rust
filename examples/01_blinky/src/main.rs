#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use nucleo_bsp::BoardLeds;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("==========================================");
    info!("NUCLEO-H743ZI2 Embassy Blinky (Workspace)");
    info!("==========================================");

    // STM32H743 기본 주변장치 클럭 초기화
    let p = embassy_stm32::init(Default::default());

    // BSP의 BoardLeds를 통한 온보드 사용자 LED 세트 초기화
    let mut leds = BoardLeds::new(p.PB0, p.PE1, p.PB14);
    info!("BSP BoardLeds 초기화 완료 (LED1: PB0, LED2: PE1, LED3: PB14)");

    let mut counter: u32 = 0;

    loop {
        counter += 1;
        info!("Blink Cycle #{}: 순차 점멸 시작", counter);

        // 1. Green LED (PB0)
        leds.green.set_high();
        Timer::after_millis(300).await;
        leds.green.set_low();

        // 2. Yellow LED (PE1)
        leds.yellow.set_high();
        Timer::after_millis(300).await;
        leds.yellow.set_low();

        // 3. Red LED (PB14)
        leds.red.set_high();
        Timer::after_millis(300).await;
        leds.red.set_low();

        Timer::after_millis(500).await;
    }
}
