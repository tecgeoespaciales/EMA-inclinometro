import ujson

def load_calibration(filename="mag_calibration.json"):
    try:
        with open(filename, "r") as f:
            calib = ujson.load(f)
        print("Calibración cargada correctamente.")
        return calib
    except Exception as e:
        print("Error al cargar calibración:", e)
        return None

def apply_calibration(raw_x, raw_y, raw_z, calib):
    # Aplica solo offset; puedes agregar escala si es necesario
    x_corr = raw_x - calib["offset"]["x"]
    y_corr = raw_y - calib["offset"]["y"]
    z_corr = raw_z - calib["offset"]["z"]
    return x_corr, y_corr, z_corr