def negativeNumberHandler(x: int):
    if x < 0:
        x = abs(x) + 100
    return x
