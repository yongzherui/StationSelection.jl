using Logging
using Printf

const T0 = time()

# Function to format log level
function level_str(level::Logging.LogLevel)
    s = uppercase(string(level))
    return lpad(s, 5)
end

mutable struct StationSelectionLogger <: Logging.AbstractLogger
    stream::IO
    file_stream::Union{Nothing, IO}
    min_level::Logging.LogLevel
    message_limits::Dict{Any, Int}
end

StationSelectionLogger(stream::IO=stderr; min_level::Logging.LogLevel=Logging.Info, file_stream::Union{Nothing, IO}=nothing) =
    StationSelectionLogger(stream, file_stream, min_level, Dict{Any, Int}())

Logging.min_enabled_level(logger::StationSelectionLogger) = logger.min_level
Logging.shouldlog(logger::StationSelectionLogger, level, _module, group, id) =
    get(logger.message_limits, id, 1) > 0
Logging.catch_exceptions(logger::StationSelectionLogger) = false

function Logging.handle_message(logger::StationSelectionLogger, level, message, _module, group, id,
                                file, line; kwargs...)
    elapsed = round(time() - T0; digits=2)
    elapsedcol = @sprintf("+%6.2fs", elapsed)
    formatted_msg = "[$elapsedcol] $(level_str(level))  $(String(message))"
    if !isempty(kwargs)
        kwargs_str = join(["$k=$v" for (k, v) in kwargs], ", ")
        formatted_msg *= " ($kwargs_str)"
    end

    println(logger.stream, formatted_msg)
    flush(logger.stream)

    if logger.file_stream !== nothing
        println(logger.file_stream, formatted_msg)
        flush(logger.file_stream)
    end

    nothing
end

function setup_station_selection_logging!(; min_level::Logging.LogLevel=Logging.Info, stream::IO=stderr, log_file::Union{Nothing, String}=nothing)
    file_stream = if log_file !== nothing
        open(log_file, "w")
    else
        nothing
    end
    logger = StationSelectionLogger(stream, min_level=min_level, file_stream=file_stream)
    global_logger(logger)
    return logger
end

function close_station_selection_logging!()
    logger = Logging.global_logger()
    if logger isa StationSelectionLogger && logger.file_stream !== nothing
        close(logger.file_stream)
    end
end

setup_station_selection_logging!()
