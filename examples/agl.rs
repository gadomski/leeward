use clap::{App, Arg};
use csv::Writer;
use las::{Read, Reader};
use leeward::Trajectory;
use serde::Serialize;
use std::collections::HashMap;

#[derive(Debug, Serialize)]
struct CsvPoint {
    easting: f64,
    northing: f64,
    altitude: f64,
    agl: f64,
}

fn main() {
    let matches = App::new("trajectory")
        .arg(Arg::with_name("sbet").help("sbet file").index(1))
        .arg(Arg::with_name("utm-zone").help("utm zone").index(2))
        .arg(Arg::with_name("outfile").help("out csv file").index(3))
        .arg(Arg::from_usage("<las>... 'one or more las files'"))
        .get_matches();
    let utm_zone: u8 = matches.value_of("utm-zone").unwrap().parse().unwrap();
    println!("Reading trajectory");
    let trajectory: HashMap<(i64, i64), sbet::Point> =
        Trajectory::from_path(matches.value_of("sbet").unwrap())
            .unwrap()
            .points()
            .iter()
            .map(|p| {
                let (northing, easting, _) =
                    utm::radians_to_utm_wgs84(p.latitude, p.longitude, utm_zone);
                ((northing.round() as i64, easting.round() as i64), *p)
            })
            .collect();
    let outfile = matches.value_of("outfile").unwrap();
    let mut writer = Writer::from_path(outfile).unwrap();
    let mut points = HashMap::new();
    for las in matches.values_of("las").unwrap() {
        let mut reader = Reader::from_path(las).unwrap();
        println!("Reading {}", las);
        for point in reader.points() {
            let point = point.unwrap();
            let key = (point.y.round() as i64, point.x.round() as i64);
            if trajectory.contains_key(&key) {
                points.entry(key).or_insert(point);
            }
        }
    }
    println!("trajectory={}, points={}", trajectory.len(), points.len());
    println!("Writing {}", outfile);
    for (key, value) in trajectory {
        if let Some(point) = points.get(&key) {
            let (northing, easting, _) =
                utm::radians_to_utm_wgs84(value.latitude, value.longitude, utm_zone);
            let agl = value.altitude - point.z;
            let csv_point = CsvPoint {
                northing,
                easting,
                altitude: value.altitude,
                agl,
            };
            writer.serialize(csv_point).unwrap();
        }
    }
}
