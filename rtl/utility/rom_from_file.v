`timescale 1ns / 1ps

module rom_from_file#(
    parameter integer addr_width = 32,
    parameter integer data_width = 32,
    parameter file = "program_rom.mem",
    parameter integer word_count = 2048)(
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 rom ADDR" *)
    input wire [0:addr_width-1] rom_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 rom CLK" *)
    input wire rom_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 rom DOUT" *)
    output reg [0:data_width-1] rom_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 rom EN" *)
    input wire rom_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 rom RST" *)
    input wire rom_rst
    );

    reg [0:7] bytes[0:(data_width / 8) * word_count - 1];

    initial begin
        $readmemh(file, bytes, 0, (data_width / 8) * word_count);
    end

    genvar i;
    genvar j;
    generate
        for (i = 0; i < data_width / 8; i = i + 1) begin
            always @(posedge rom_clk) begin
                if (rom_en) begin
                    if (rom_addr + i > word_count) begin
                        rom_din[8 * i+:8] <= 8'b0;
                    end else begin
                        rom_din[8 * i+:8] <= bytes[rom_addr + i];
                    end
                end
            end
        end
    endgenerate

endmodule