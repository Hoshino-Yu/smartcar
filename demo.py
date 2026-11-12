from control.line_follower import LineFollower


def main() -> None:
    follower = LineFollower()
    samples = [
        ("center", (0, 0, 1, 0, 0)),
        ("left", (0, 1, 0, 0, 0)),
        ("right", (0, 0, 0, 1, 0)),
        ("lost", (0, 0, 0, 0, 0)),
        ("center", (0, 0, 1, 0, 0)),
    ]
    for label, readings in samples:
        left, right = follower.step(readings)
        print(f"{label:>6}  {readings}  left={left:.2f}  right={right:.2f}")


if __name__ == "__main__":
    main()
