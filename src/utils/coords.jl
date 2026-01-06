module CoordTransform

# Constants
const x_pi = π * 3000.0 / 180.0
const a = 6378245.0
const ee = 0.00669342162296594323

"Check if coordinates are outside China"
function out_of_china(lat::Float64, lon::Float64)::Bool
    return lon < 72.004 || lon > 137.8347 || lat < 0.8293 || lat > 55.8271
end

"Transform latitude"
function transform_lat(x::Float64, y::Float64)::Float64
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y^2 + 0.1 * x * y + 0.2 * sqrt(abs(x))
    ret += (20.0 * sin(6.0 * x * π) + 20.0 * sin(2.0 * x * π)) * 2.0 / 3.0
    ret += (20.0 * sin(y * π) + 40.0 * sin(y / 3.0 * π)) * 2.0 / 3.0
    ret += (160.0 * sin(y / 12.0 * π) + 320.0 * sin(y * π / 30.0)) * 2.0 / 3.0
    return ret
end

"Transform longitude"
function transform_lon(x::Float64, y::Float64)::Float64
    ret = 300.0 + x + 2.0 * y + 0.1 * x^2 + 0.1 * x * y + 0.1 * sqrt(abs(x))
    ret += (20.0 * sin(6.0 * x * π) + 20.0 * sin(2.0 * x * π)) * 2.0 / 3.0
    ret += (20.0 * sin(x * π) + 40.0 * sin(x / 3.0 * π)) * 2.0 / 3.0
    ret += (150.0 * sin(x / 12.0 * π) + 300.0 * sin(x / 30.0 * π)) * 2.0 / 3.0
    return ret
end

"BD-09LL -> GCJ-02"
function bd09_to_gcj02(bd_lon::Float64, bd_lat::Float64)
    x = bd_lon - 0.0065
    y = bd_lat - 0.006
    z = sqrt(x^2 + y^2) - 0.00002 * sin(y * x_pi)
    theta = atan(y, x) - 0.000003 * cos(x * x_pi)
    gcj_lon = z * cos(theta)
    gcj_lat = z * sin(theta)
    return gcj_lon, gcj_lat
end

"GCJ-02 -> WGS84"
function gcj02_to_wgs84(gcj_lon::Float64, gcj_lat::Float64)
    if out_of_china(gcj_lat, gcj_lon)
        return gcj_lon, gcj_lat
    end

    dlat = transform_lat(gcj_lon - 105.0, gcj_lat - 35.0)
    dlon = transform_lon(gcj_lon - 105.0, gcj_lat - 35.0)
    rad_lat = gcj_lat / 180.0 * π
    magic = sin(rad_lat)
    magic = 1.0 - ee * magic * magic
    sqrt_magic = sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1.0 - ee)) / (magic * sqrt_magic) * π)
    dlon = (dlon * 180.0) / (a / sqrt_magic * cos(rad_lat) * π)
    wgs_lat = gcj_lat - dlat
    wgs_lon = gcj_lon - dlon
    return wgs_lon, wgs_lat
end

"BD-09LL -> WGS84"
function bd09_to_wgs84(bd_lon::Float64, bd_lat::Float64)
    gcj_lon, gcj_lat = bd09_to_gcj02(bd_lon, bd_lat)
    return gcj02_to_wgs84(gcj_lon, gcj_lat)
end

export bd09_to_wgs84, bd09_to_gcj02, gcj02_to_wgs84

end # module
