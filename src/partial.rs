use std::fmt;

/// A partial derivative.
#[derive(Debug, Clone, Copy)]
pub struct Partial(Dimension, Variable);

/// A dimension, as used for partial derivatives.
#[derive(Debug, Clone, Copy)]
pub enum Dimension {
    X,
    Y,
    Z,
}

/// A variable in the lidar equation.
#[derive(Debug, Clone, Copy)]
pub enum Variable {
    Range,
    ScanAngle,
    BoresightRoll,
    BoresightPitch,
    BoresightYaw,
    LeverArmX,
    LeverArmY,
    LeverArmZ,
    ImuRoll,
    ImuPitch,
    ImuYaw,
    GnssX,
    GnssY,
    GnssZ,
}

impl Partial {
    /// Returns a vector of all partial derivatives.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Partial;
    /// assert_eq!(14 * 3, Partial::all().len());
    /// ```
    pub fn all() -> Vec<Partial> {
        let mut all = Vec::new();
        for variable in Variable::all() {
            for dimension in Dimension::all() {
                all.push(Partial(dimension, variable));
            }
        }
        all
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

impl fmt::Display for Partial {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "d{}/d{}", self.0, self.1)
    }
}

impl Variable {
    /// Returns a vector of all variables.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Variable;
    /// assert_eq!(14, Variable::all().len());
    /// ```
    pub fn all() -> Vec<Variable> {
        vec![
            Variable::Range,
            Variable::ScanAngle,
            Variable::BoresightRoll,
            Variable::BoresightPitch,
            Variable::BoresightYaw,
            Variable::LeverArmX,
            Variable::LeverArmY,
            Variable::LeverArmZ,
            Variable::ImuRoll,
            Variable::ImuPitch,
            Variable::ImuYaw,
            Variable::GnssX,
            Variable::GnssY,
            Variable::GnssZ,
        ]
    }

    /// Returns true if this variable is an angle, false if not.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Variable;
    /// assert!(Variable::BoresightRoll.is_angle());
    /// assert!(!Variable::LeverArmX.is_angle());
    /// ```
    pub fn is_angle(&self) -> bool {
        match *self {
            Variable::ScanAngle
            | Variable::BoresightRoll
            | Variable::BoresightPitch
            | Variable::BoresightYaw
            | Variable::ImuRoll
            | Variable::ImuPitch
            | Variable::ImuYaw => true,
            _ => false,
        }
    }
}

impl fmt::Display for Variable {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Variable::Range => "Range",
                Variable::ScanAngle => "ScanAngle",
                Variable::BoresightRoll => "BoresightRoll",
                Variable::BoresightPitch => "BoresightPitch",
                Variable::BoresightYaw => "BoresightYaw",
                Variable::LeverArmX => "LeverArmX",
                Variable::LeverArmY => "LeverArmY",
                Variable::LeverArmZ => "LeverArmZ",
                Variable::ImuRoll => "ImuRoll",
                Variable::ImuPitch => "ImuPitch",
                Variable::ImuYaw => "ImuYaw",
                Variable::GnssX => "GnssX",
                Variable::GnssY => "GnssY",
                Variable::GnssZ => "GnssZ",
            }
        )
    }
}

impl Dimension {
    /// Returns a vector of all dimensions.
    ///
    /// # Examples
    ///
    /// ```
    /// # use leeward::Dimension;
    /// assert_eq!(3, Dimension::all().len());
    /// ```
    pub fn all() -> Vec<Dimension> {
        vec![Dimension::X, Dimension::Y, Dimension::Z]
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
