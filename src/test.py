def calculate(target):
    if target == 640:
        return 0
    direction = -1 if target < 640 else 1
    ratio = abs(target - 640) / 640
    return direction * (10 * ratio)

for i in range(620, 660):
    print(i, calculate(i))

print(calculate(0))
print(calculate(1280))
