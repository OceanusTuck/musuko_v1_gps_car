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
        if (Goto(13.867608782221238, 100.28452270522767, 2) == 1) {
            Musuko.stop_all_wheel()
            is_start = 0
            basic.showIcon(IconNames.Yes)
        }
    }
})
