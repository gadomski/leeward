use clap::{App, Arg};

fn main() {
    let matches = App::new("measurement")
        .arg(
            Arg::with_name("sbet")
                .help("sbet file")
                .required(true)
                .index(1),
        )
        .arg(
            Arg::with_name("las")
                .help("las file")
                .required(true)
                .index(2),
        )
        .arg(
            Arg::with_name("config")
                .help("config file")
                .required(true)
                .index(3),
        )
        .arg(
            Arg::with_name("number")
                .help("measurement number to print")
                .required(true)
                .index(4),
        )
        .get_matches();
    let number: usize = matches.value_of("number").unwrap().parse().unwrap();
    let measurements = leeward::measurements(
        matches.value_of("sbet").unwrap(),
        matches.value_of("las").unwrap(),
        matches.value_of("config").unwrap(),
    )
    .unwrap();
    let measurement = &measurements[number];
    println!(
        "{}",
        toml::to_string_pretty(&measurement.variables().unwrap()).unwrap()
    );
}
