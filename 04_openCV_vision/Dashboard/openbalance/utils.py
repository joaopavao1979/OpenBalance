# --------------------------------------------------
# File: utils.py
# --------------------------------------------------

def map_value(value, src_min, src_max, dst_min, dst_max):
    """
    Converte 'value' do intervalo [src_min, src_max] para [dst_min, dst_max].
    """
    return (value - src_min) * (dst_max - dst_min) / (src_max - src_min) + dst_min
