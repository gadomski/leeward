use anyhow::Error;
use clap::{load_yaml, App, AppSettings};

fn main() -> Result<(), Error> {
    let yaml = load_yaml!("cli.yml");
    let matches = App::from_yaml(yaml)
        .setting(AppSettings::DisableHelpSubcommand)
        .get_matches();
    let _measurements = leeward::measurements_with_decimation(
        matches.value_of("SBET").unwrap(),
        matches.value_of("LAS").unwrap(),
        matches.value_of("CONFIG").unwrap(),
        matches
            .value_of("decimation")
            .map(|v| v.parse())
            .transpose()?
            .unwrap_or(1),
    );
    if let Some(_matches) = matches.subcommand_matches("adjust") {
        unimplemented!()
    } else if let Some(_matches) = matches.subcommand_matches("body_frame") {
        unimplemented!()
    }
    Ok(())
}
