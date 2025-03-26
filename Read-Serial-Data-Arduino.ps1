param(
    [Parameter(Mandatory=$true)][string]$PortName,
    [Parameter(Mandatory=$true)][int]$BaudRate,
    [Parameter()][string]$CsvPath = ""
)

# Initialize serial port communication
$port = New-Object System.IO.Ports.SerialPort
$port.PortName = $PortName
$port.BaudRate = $BaudRate
$port.Parity = [System.IO.Ports.Parity]::None
$port.DataBits = 8
$port.StopBits = [System.IO.Ports.StopBits]::One
$port.Handshake = [System.IO.Ports.Handshake]::None
$port.ReadTimeout = 10000 # milliseconds

$port.ReadBufferSize = 16384 # bytes
# Can handle ~1.4 seconds of buffered data at baud rate of 115200
# Or ~17 seconds of buffered data at baud rate of 9600

# Supported Baud Rates: 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 31250, 38400, 57600, 115200
# Formula: Seconds of data = (Buffer Size) / (Baud Rate / 10)

$port.DtrEnable = "true"
# This allows sending the "Data Terminal Ready Enabled", which is necessary (somehow) for reading serial data.
# Apparently it's used for auto-resetting the Arduino, although this script doesn't do that from basic testing.

try {
    $port.Open()
    Write-Host "Opened serial port $($port.PortName) using baud rate $($port.BaudRate)"

    if ($CsvPath.length -gt 0) {
        # Create a file writer handle
        $FullCsvPath = $(Join-Path (Resolve-Path .) $CsvPath)
        Write-Host $FullCsvPath
        $CsvWriter = [System.IO.StreamWriter]::new($FullCsvPath, $true)
        Write-Host "All data will be written to $CsvPath in comma-delimited CSV format"
    }

    # Discard any existing data in the buffer
    $port.DiscardInBuffer()

    # Read data continuously until an error occurs or no more data is available
    while ($port.IsOpen) {
        try {
            # Record starting read time
            $StartReadTime = Get-Date
            $SerialData = $port.ReadExisting()

            if ($CsvPath.length -gt 0) {
                $CsvWriter.Write($SerialData)
            }

            # Stream received data
            Write-Host $SerialData -NoNewLine
        } catch [System.TimeoutException] {
            Write-Host "Error: Did not receive any data after $($port.ReadTimeout / 1000) seconds"
        }
    }
} catch {
    Write-Host "Error: $($_.Exception.Message)"
} finally {
    # Close the CSV file writer (if present)
    if ($CsvWriter -ne $null) {
        $CsvWriter.Close()
        $CsvWriter.Dispose()
    }

    # Close the serial port and dispose of the handle
    if ($port.IsOpen) {
        $port.Close()
        $port.Dispose()
        Write-Host "Closed serial port $($port.PortName)"
    }
}
