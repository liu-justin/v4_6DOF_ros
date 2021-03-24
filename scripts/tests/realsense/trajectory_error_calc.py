def linear(x_1, x_2, t_1, t_2, x_error, t_error):
    x1_low = x_1(1-x_error)
    x1_high = x_1(1+x_error)
    t1_low = t_1(1-t_error)
    t1_high = t_1(1+t_error)
    x2_low = x_2(1-x_error)
    x2_high = x_2(1+x_error)
    t2_low = t_2(1-t_error)
    t2_high = t_2(1+t_error)
    # full negative slope
    if (x2_high - x1_low) <= 0:
        min_m = (x2_low - x1_high)/(t2_low - t1_high) # BL2_TR1_m
        max_m = (x2_high - x1_low)/(t2_high - t1_low) # TR2_BL1_m
        min_b = x1_high - min_m*t1_high
        max_b = x1_low - max_m*t1_low

    
    # mixed slope
    elif (x2_low - x1_high) <= 0:
        min_m = (x2_low - x1_high)/(t2_low - t1_high) # BL2_TR1_m
        max_m = (x2_high - x1_low)/(t2_low - t1_high) # TL2_BR1_m
        min_b = x1_high - min_m*t1_high
        max_b = x1_low - max_m*t1_high

    # all positive slope
    else:
        min_m = (x2_low - x1_high)/(t2_high - t1_low) # BR2_TL1_m
        max_m = (x2_high - x1_low)/(t2_low - t1_high) # TL2_BR1_m
        min_b = x1_high - min_m*t1_low
        max_b = x1_low - max_m*t1_high

    return min_m, min_b, max_m, max_b


    # TL2_BR1_m = (x2_high - x1_low)/(t2_low - t1_high)
    # BR2_TL1_m = (x2_low - x1_high)/(t2_high - t1_low)
    # TR2_BL1_m = (x2_high - x1_low)/(t2_high - t1_low)
    # BL2_TR1_m = (x2_low - x1_high)/(t2_low - t1_high)

# https://stackoverflow.com/questions/21565994/method-to-return-the-equation-of-a-straight-line-given-two-points
# polyfit uses least squares fit, lets see which one is faster