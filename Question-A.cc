// Part A: This is an extension task that requires you to decode sensor data from CAN log files.
// CAN (Controller Area Network) is a communication standard used in automotive applications (including Redback cars)
// to allow communication between sensors and controllers.
//
// Your Task: Using the signal definitions in SteeringBench.dbc, read each CAN capture in data/
// and turn it into a CSV with one row per decoded frame:
// t,u_commanded,y_measured
// eg:
// 0,15.0,0.0
// 0.005,15.0,0.0
// ...
// where t is the frame timestamp minus the first kept frame's timestamp (s), u_commanded is
// the decoded CmdAngularRate (deg/s), and y_measured is the decoded MeasuredAngle (deg).
// The above values are not real numbers; they are only there to show the expected data output format.
// Do this for all three captures:
// data/step_test.log       ->  data/step_test.csv
// data/reversal_test.log   ->  data/reversal_test.csv
// data/deadband_test.log   ->  data/deadband_test.csv
//
// The Row type, writeCsv(), and main() below are provided -- they loop the three logs, call your
// decodeLog(), and write the CSV in exactly the format above. You just need to implement decodeLog().
//
// You do not need to use any external libraries. Use the resources below to understand how to
// extract sensor data.
// Hint: Think about manual bit masking and shifting, data types required,
// what formats are used to represent values, etc.
// Resources:
// https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial
// https://www.csselectronics.com/pages/can-dbc-file-database-intro
//
// Sanity check: plot your CSVs (python3 plot_data.py) and compare against the pre-plotted
// data/*.png files -- they should match.
//
// Build & run (from the TA/ folder):
//     c++ -std=c++17 Question-A.cc -o decode
//     ./decode

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>


// One output row.
struct Row {
    double t;            // seconds since the first kept frame
    double u_commanded;  // deg/s
    double y_measured;   // deg
};

// Read the candump log at `path` and return one Row per STEER_ActuatorLog frame, in order.
// Push one Row{t, u_commanded, y_measured} per kept frame.
std::vector<Row> decodeLog(const std::string& path) {
    std::vector<Row> rows;

    std::ifstream in(path);
    if (!in) {
        std::printf("Could not open %s\n", path.c_str());
        return rows;
    }

    bool haveFirst = false;
    double t0 = 0.0;
    std::string line;

    while (std::getline(in, line)) {
        if (line.empty()) continue;

        // Pull the timestamp out of "(...)"
        size_t lp = line.find('(');
        size_t rp = line.find(')');
        if (lp == std::string::npos || rp == std::string::npos || rp < lp) continue;
        double timestamp = std::stod(line.substr(lp + 1, rp - lp - 1));

        // Find "ID#DATA"
        size_t hash = line.find('#', rp);
        if (hash == std::string::npos) continue;
        size_t idStart = line.rfind(' ', hash);
        if (idStart == std::string::npos) continue;
        idStart += 1;
        std::string idStr = line.substr(idStart, hash - idStart);
        unsigned long id = std::stoul(idStr, nullptr, 16);

        if (id != 512) continue;  // 0x200 -- only keep STEER_ActuatorLog frames

        // Grab the hex payload after '#', trim trailing junk (CR, spaces, flags)
        std::string dataStr = line.substr(hash + 1);
        while (!dataStr.empty()) {
            char c = dataStr.back();
            bool isHex = (c >= '0' && c <= '9') ||
                         (c >= 'a' && c <= 'f') ||
                         (c >= 'A' && c <= 'F');
            if (isHex) break;
            dataStr.pop_back();
        }
        if (dataStr.size() < 16) continue;  // need 8 bytes = 16 hex chars

        unsigned char bytes[8];
        for (int i = 0; i < 8; ++i)
            bytes[i] = static_cast<unsigned char>(std::stoul(dataStr.substr(i * 2, 2), nullptr, 16));

        // Little-endian 16-bit: first byte is the low byte
        int rawMeasured = bytes[0] | (bytes[1] << 8);
        int rawCmdRate  = bytes[2] | (bytes[3] << 8);

        // Signed 16-bit: values >= 0x8000 are negative
        if (rawMeasured >= 0x8000) rawMeasured -= 0x10000;
        if (rawCmdRate  >= 0x8000) rawCmdRate  -= 0x10000;

        double y_measured  = rawMeasured * 0.1;  // deg
        double u_commanded = rawCmdRate  * 0.1;  // deg/s

        if (!haveFirst) { t0 = timestamp; haveFirst = true; }

        rows.push_back(Row{timestamp - t0, u_commanded, y_measured});
    }

    return rows;
}


    // TODO: your code here
    /*
    Pseudocoding time!

    Using the data log, we need to:
    a. 'parse' the timestamp 
    b. 'parse' the canid

    Go to to deadband to see the neccesary hex value; here 512 (base 10) can be converted to 200 in hex
    if the hex code is not 0x200 (which is 512), then skip it as it is not required here
    Now, all that is required is for the specific range of bits, e.g 0-16, 16-32, 32-48, 48-64 for each variable
    
    finding and decrypting 'parsing' the bytes will look like this:

    @1 means little-endian, so the first byte is the low byte. That's why you shift b1, not b0.
    The second number in 0|16 is the length in bits, not related to @1.
    for deg and deg/s the important ones are the first four bytes. We know this because 
    MeasuredAngle  : 0|16   -> bits 0-15   -> bytes 0 and 1  (b0, b1)
    CmdAngularRate : 16|16  -> bits 16-31  -> bytes 2 and 3  (b2, b3)

    next, we need to know:
    Which signals do I need?   -> MeasuredAngle, CmdAngularRate
    Where are they (per DBC)?  -> bytes 0-1 and bytes 2-3
    Therefore                  -> b0 to b3 are needed

    Understanding with the help of Claude, for example:
    b0            =           0110 0100   (0x64 = 100)
    b1 << 8       = 0000 0001 0000 0000   (0x01 shifted up = 256)
    b0 | (b1<<8)  = 0000 0001 0110 0100   (0x0164 = 356)

    for the measured angle:
    y_measured = raw * 0.1

    for the CmdAngularRate:
    u_commanded = raw * 0.1

    once done, if this is the first kept frame: t0 = timestamp
    t = timestamp - t0
    store Row{t, u_commanded, y_measured} -> basically what is doing the final compiling.
    */
    // (void)path;  // remove once you open the file

// Provided -- writes the rows to a CSV in the required format. Do not change.
void writeCsv(const std::string& path, const std::vector<Row>& rows) {
    std::ofstream f(path);
    f << "t,u_commanded,y_measured\n";
    for (const Row& r : rows)
        f << r.t << "," << r.u_commanded << "," << r.y_measured << "\n";
}

// Provided -- runs decodeLog() + writeCsv() for each of the three captures.
int main() {
    const char* names[] = {"step_test", "reversal_test", "deadband_test"};
    for (const char* n : names) {
        const std::string in  = std::string("data/") + n + ".log";
        const std::string out = std::string("data/") + n + ".csv";
        const std::vector<Row> rows = decodeLog(in);
        writeCsv(out, rows);
        std::printf("%-14s %6zu frames -> %s\n", n, rows.size(), out.c_str());
    }
    return 0;
}
