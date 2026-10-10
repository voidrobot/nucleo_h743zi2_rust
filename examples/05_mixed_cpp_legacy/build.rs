use std::env;

fn main() {
    let target = env::var("TARGET").unwrap_or_default();
    let mut build = cc::Build::new();

    build.cpp(true);
    build.cpp_link_stdlib(None);
    build.file("cpp/src/biquad_filter.cpp");
    build.include("cpp/include");
    build.flag_if_supported("-std=c++17");
    build.flag_if_supported("-fno-exceptions");
    build.flag_if_supported("-fno-rtti");
    build.flag_if_supported("-fno-unwind-tables");
    build.opt_level(3);

    // 타깃 아키텍처가 ARM Cortex-M7 (thumbv7em-none-eabihf)인 경우
    if target.contains("thumbv7em") || target.contains("arm") {
        // ARM GCC 툴체인(arm-none-eabi-g++)을 기본 컴파일러로 지정하여 전체 예제 툴체인 일관성 유지
        build.compiler("arm-none-eabi-g++");
        build.target(&target);
        build.flag("-mcpu=cortex-m7");
        build.flag("-mfpu=fpv5-d16");
        build.flag("-mfloat-abi=hard");
        build.flag("-mthumb");
    }

    build.compile("legacy_dsp");

    println!("cargo:rerun-if-changed=cpp/include/biquad_filter.hpp");
    println!("cargo:rerun-if-changed=cpp/src/biquad_filter.cpp");
}
