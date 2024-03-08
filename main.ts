function Goto (lat: number, lon: number, error: number) {
    if (Musuko.distanceTo(lat, lon) > error) {
        if (Musuko.angleDifferenceLeftRight(input.compassHeading(), Musuko.angleBetweenCoordinates(lat, lon)) > 10) {
            Musuko.wheel_run(Motor_Ch.M3, Wheel_Dir.Front, 70)
        } else if (Musuko.angleDifferenceLeftRight(input.compassHeading(), Musuko.angleBetweenCoordinates(lat, lon)) < -10) {
            Musuko.wheel_run(Motor_Ch.M3, Wheel_Dir.Back, 70)
        } else {
            Musuko.wheel_run(Motor_Ch.M3, Wheel_Dir.Front, 0)
        }
        Musuko.wheel_run(Motor_Ch.M1, Wheel_Dir.Front, 100)
        Musuko.wheel_run(Motor_Ch.M2, Wheel_Dir.Front, 100)
        return 0
    } else {
        return 1
    }
    return 0
}
input.onButtonPressed(Button.A, function () {
    basic.showIcon(IconNames.Triangle)
    is_start = 1
})
input.onButtonPressed(Button.B, function () {
    basic.showIcon(IconNames.Heart)
    Musuko.stop_all_wheel()
    is_start = 0
})
let is_start = 0
Musuko.gps_setup()
is_start = 0
basic.showIcon(IconNames.Heart)
basic.forever(function () {
    Musuko.gps_update()
})
basic.forever(function () {
    if (is_start == 1) {
        if (Goto(13.798286268293245, 100.32055894006942, 2) == 1) {
            is_start = 2
            basic.showNumber(1)
        }
    } else if (is_start == 2) {
        if (Goto(13.797976063264928, 100.32056003774267, 2) == 1) {
            is_start = 3
            basic.showNumber(2)
        }
    } else if (is_start == 3) {
        if (Goto(13.797985657241071, 100.32082896769316, 2) == 1) {
            is_start = 4
            basic.showNumber(3)
        }
    } else if (is_start == 4) {
        if (Goto(13.798307588211303, 100.32080811190109, 2) == 1) {
            is_start = 5
            basic.showNumber(4)
        }
    } else if (is_start == 5) {
        if (Goto(13.798286268293245, 100.32055894006942, 2) == 1) {
            is_start = 6
            basic.showIcon(IconNames.House)
        }
    } else {
        Musuko.stop_all_wheel()
        is_start = 0
        basic.showIcon(IconNames.Yes)
    }
})
