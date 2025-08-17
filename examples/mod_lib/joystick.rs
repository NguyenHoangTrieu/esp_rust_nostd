pub fn get_direct(x_axis: u16, y_axis: u16) -> u32{
    if x_axis < 1800 && y_axis > 3300{ return 1;}
    else if x_axis > 4090 && y_axis > 3300 {return 2;}
    else if y_axis < 1800 && x_axis > 3300 {return 3;}
    else if y_axis > 4090 && x_axis > 3300 {return 4;}
    else {return 0;}
}