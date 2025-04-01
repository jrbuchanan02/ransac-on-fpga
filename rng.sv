`timescale 1ns / 1ps

module lfsr#(
        parameter bit [63:0] seed = 64'h5A5A_A5A5_B4B4_4B4B
    )(
        input logic rng_clock,
        input logic rng_reset,
        input logic read_clock,
        output logic [63:0] random_value
    );

    logic [63:0] r;

    (* ASYNC_REG = "TRUE" *) reg [63:0] sync_regs[2:0];

    assign random_value = sync_regs[2];

    always_ff @(posedge read_clock) begin
        sync_regs[0] <= r[63:0];
        for (int i = 0; i < 2; i++) begin
            sync_regs[i+1] <= sync_regs[i];
        end
    end

    always_ff @(posedge rng_clock) begin
        if (rng_reset || r == 0 || r == {64{1'b1}}) begin
            r <= seed;
        end else begin
            r[63:1] <= r[62:0];
            r[0] <= r[63] ~^ r[62] ~^ r[59] ~^ r[58];
        end
    end

endmodule : lfsr