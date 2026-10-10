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
        // Clang 18 크로스 컴파일러 우선 지정
        let compiler_candidates = ["clang++-18", "clang++", "arm-none-eabi-g++"];
        for c in compiler_candidates {
            if which_compiler_exists(c) {
                build.compiler(c);
                break;
            }
        }

        build.target(&target);
        build.flag("-mcpu=cortex-m7");
        build.flag("-mfpu=fpv5-d16");
        build.flag("-mfloat-abi=hard");
    }

    build.compile("legacy_dsp");

    println!("cargo:rerun-if-changed=cpp/include/biquad_filter.hpp");
    println!("cargo:rerun-if-changed=cpp/src/biquad_filter.cpp");
}

fn which_compiler_exists(cmd: &str) -> bool {
    std::process::Command::new("which")
        .arg(cmd)
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}
