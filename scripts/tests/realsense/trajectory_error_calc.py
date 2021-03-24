def linear(x_1, x_2, t_1, t_2, x_error, t_error):
    x_1_low = x_1(1-x_error)
    x_1_high = x_1(1+x_error)
    t_1_low = t_1(1-t_error)
    t_1_high = t_1(1+t_error)
    x_2_low = x_2(1-x_error)
    x_2_high = x_2(1+x_error)
    t_2_low = t_2(1-t_error)
    t_2_high = t_2(1+t_error)
    TL2_BR1_m = (x_2_high - x_1_low)/(t_2_low - t_1_high)
    BR2_TL1_m = (x_2_low - x_1_high)/(t_2_high - t_1_low)
    TR2_BL1_m = (x_2_high - x_1_low)/(t_2_high - t_1_low)
    BL2_TR1_m = (x_2_low - x_1_high)/(t_2_low - t_1_high)

    lower_m = min(TL2_BR1_m,BR2_TL1_m,TR2_BL1_m,BL2_TR1_m)
    higher_m = max(TL2_BR1_m,BR2_TL1_m,TR2_BL1_m,BL2_TR1_m)

# https://stackoverflow.com/questions/21565994/method-to-return-the-equation-of-a-straight-line-given-two-points
# polyfit uses least squares fit, lets see which one is faster