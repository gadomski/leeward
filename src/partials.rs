use std::fmt;

/// A dimension of a lidar measurement.
#[derive(Clone, Copy, Debug)]
pub enum Dimension {
    X,
    Y,
    Z,
}

/// A variable of a lidar measurement.
#[derive(Clone, Copy, Debug)]
pub enum Variable {
    GnssX,
    GnssY,
    GnssZ,
    ImuRoll,
    ImuPitch,
    ImuYaw,
    BoresightRoll,
    BoresightPitch,
    BoresightYaw,
    Distance,
    ScanAngle,
    LeverArmX,
    LeverArmY,
    LeverArmZ,
}

/// A combination of a variable and a dimension for a partial.
#[derive(Debug, Clone, Copy)]
pub struct Partial(pub Dimension, pub Variable);

impl Variable {
    pub fn iter() -> std::vec::IntoIter<Variable> {
        vec![
            Variable::GnssX,
            Variable::GnssY,
            Variable::GnssZ,
            Variable::ImuRoll,
            Variable::ImuPitch,
            Variable::ImuYaw,
            Variable::BoresightRoll,
            Variable::BoresightPitch,
            Variable::BoresightYaw,
            Variable::Distance,
            Variable::ScanAngle,
            Variable::LeverArmX,
            Variable::LeverArmY,
            Variable::LeverArmZ,
        ]
        .into_iter()
    }
}

impl fmt::Display for Variable {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Variable::GnssX => "GnssX",
                Variable::GnssY => "GnssY",
                Variable::GnssZ => "GnssZ",
                Variable::ImuRoll => "ImuRoll",
                Variable::ImuPitch => "ImuPitch",
                Variable::ImuYaw => "ImuYaw",
                Variable::BoresightRoll => "BoresightRoll",
                Variable::BoresightPitch => "BoresightPitch",
                Variable::BoresightYaw => "BoresightYaw",
                Variable::Distance => "Distance",
                Variable::ScanAngle => "ScanAngle",
                Variable::LeverArmX => "LeverArmX",
                Variable::LeverArmY => "LeverArmY",
                Variable::LeverArmZ => "LeverArmZ",
            }
        )
    }
}

impl Dimension {
    pub fn iter() -> std::vec::IntoIter<Dimension> {
        vec![Dimension::X, Dimension::Y, Dimension::Z].into_iter()
    }
}

impl fmt::Display for Dimension {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Dimension::X => "X",
                Dimension::Y => "Y",
                Dimension::Z => "Z",
            }
        )
    }
}

impl Partial {
    pub fn iter() -> std::vec::IntoIter<Partial> {
        let mut partials = vec![];
        for variable in Variable::iter() {
            for dimension in Dimension::iter() {
                partials.push(Partial(dimension, variable));
            }
        }
        partials.into_iter()
    }
}

impl fmt::Display for Partial {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "d{}/d{}", self.0, self.1)
    }
}

impl From<Partial> for (Dimension, Variable) {
    fn from(partial: Partial) -> (Dimension, Variable) {
        (partial.0, partial.1)
    }
}

impl From<(Dimension, Variable)> for Partial {
    fn from((dimension, variable): (Dimension, Variable)) -> Partial {
        Partial(dimension, variable)
    }
}
