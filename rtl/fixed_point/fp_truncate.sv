`timescale 1ns / 1ps

// truncates a fixed point number to another fixed point
// (saturates if the integer part is too great)
// the effect of this rounding is that it will always be 
// towards -infinity
//
// note: assumes that the output format is at most as precise
// as the input format
//
// note: this module may be deprecated by adding appropriate functions
// to vector_pkg.
module fp_truncate#(
        parameter int unsigned iibits = 24,
        parameter int unsigned ifbits = 40,
        parameter int unsigned oibits = 12,
        parameter int unsigned ofbits = 20,
        parameter bit signed_values = 1)(
        input logic [iibits+ifbits-1:0] ivalue,
        output logic [oibits+ofbits-1:0] ovalue
    );

    localparam logic [oibits+ofbits-1:0] unsigned_sat_value = {oibits+ofbits{1'b1}};
    localparam logic [oibits+ofbits-1:0] signed_sat_negative = { 1'b1, {oibits+ofbits-1{1'b0}}};

    logic [iibits+ofbits-1:0] rounded_down;
    logic [oibits+ofbits-1:0] truncated;

    always_comb begin
        if (signed_values) begin
            rounded_down = $signed(ivalue) >>> (ifbits - ofbits);
        end else begin
            rounded_down = $unsigned(ivalue) >>> (ifbits - ofbits);
        end
        truncated = rounded_down[oibits+ofbits-1:0];
        
        ovalue = truncated;

        if (signed_values) begin
            if ($signed(truncated) != $signed(rounded_down)) begin
                ovalue = $signed(ivalue) < 0 ? signed_sat_negative : -signed_sat_negative;
            end
        end else begin
            if ($unsigned(truncated) != $unsigned(rounded_down)) begin
                ovalue = unsigned_sat_value;
            end
        end
    end

endmodule : fp_truncate