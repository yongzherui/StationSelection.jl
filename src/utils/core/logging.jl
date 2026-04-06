using Logging
using LoggingExtras
using Dates
using Printf

const T0 = time()

# Function to format log level
function level_str(level::Logging.LogLevel)
    s = uppercase(string(level))
    return lpad(s, 5)
end

# Function to transform log messages
function add_elapsed_and_format(log)
    elapsed = round(time() - T0; digits=2)
    elapsedcol = @sprintf("+%6.2fs", elapsed)
    original_msg = String(log.message)
    formatted_msg = "[$elapsedcol] $(level_str(log.level))  $original_msg"
    return (
        level = log.level,
        message = formatted_msg,
        _module = log._module,
        group = log.group,
        id = log.id,
        file = log.file,
        line = log.line,
        kwargs = log.kwargs,
    )
end

# Create a ConsoleLogger
base_console = ConsoleLogger()

# Wrap the ConsoleLogger with the TransformerLogger
fancy_logger = TransformerLogger(add_elapsed_and_format, base_console)

# Set the global logger
global_logger(fancy_logger)
