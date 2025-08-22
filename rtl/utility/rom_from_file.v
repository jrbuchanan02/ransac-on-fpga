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
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 rom DIN" *)
    output reg [0:data_width-1] rom_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 rom EN" *)
    input wire rom_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 rom RST" *)
    input wire rom_rst
    );

    reg [0:data_width-1] words [0:word_count-1];

    wire [0:7] bytes[0:(data_width / 8) * word_count - 1];

    initial begin
        $readmemh(file, words, 0, addr_width);
    end

    genvar i;
    genvar j;
    generate
        for(i = 0; i < word_count; i = i + 1) begin
            for (j = 0; j < data_width / 8; j = j + 1) begin
                assign bytes[i + j] = words[i][8 * j+:8];
            end
        end

        for (j = 0; j < data_width / 8; j = j + 1) begin
            always @(posedge rom_clk) begin
                if (rom_en) begin
                    if (rom_addr + j > word_count) begin
                        rom_din[8 * j+:8] <= 8'b0;
                    end else begin
                        rom_din[8 * j+:8] <= bytes[rom_addr + j];
                    end
                end
            end
        end
    endgenerate

endmodule