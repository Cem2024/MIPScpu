library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.proc_config.all;

entity mipsCpu is
generic(PROG_FILE_NAME : string;
DATA_FILE_NAME : string
);
port(clk : in std_logic;
rst : in std_logic;

-- instruction insertion ports
testMode_debug : in std_logic;
testInstruction_debug : in std_logic_vector(31 downto 0);

-- ram access ports
ramInsertMode_debug : in std_logic;
ramWriteEn_debug : in std_logic;
ramWriteAddr_debug : in std_logic_vector(LOG2_NUM_RAM_ELEMENTS - 1 downto 0);
ramWriteData_debug : in std_logic_vector(RAM_ELEMENT_WIDTH - 1 downto 0);
ramElements_debug : out ram_elements_type;

-- register file access port
registers_debug : out reg_vector_type;

-- intermediate result ports
pc_next_debug : out std_logic_vector(PC_WIDTH - 1 downto 0);
pc7SegDigits_debug : out pc_7seg_digits_type
);
end mipsCpu;

architecture structural of mipsCpu is

-- Beschreibung der MIPS-CPU ergänzen

-- PC Signale
signal pc_next, pc_current : std_logic_vector (PC_WIDTH - 1 downto 0);

-- Instruction Signal

signal instruction : std_logic_vector(PC_WIDTH -1 downto 0);

-- Register Signale

signal read_register1, read_register2, write_register : std_logic_vector(LOG2_NUM_REGS-1 downto 0) := (others => '0');
signal read_data2, write_data : std_logic_vector(REG_WIDTH-1 downto 0) := (others => '0');
signal read_data1 : std_logic_vector(REG_WIDTH-1 downto 0);
signal writeEn : std_logic;

-- Alu Signale

signal alu_control : std_logic_vector(3 downto 0);
signal alu_b, alu_result : std_logic_vector(ALU_WIDTH-1 downto 0);
signal alu_zero, alu_overflow : std_logic;
signal alu_controlIn : std_logic_vector(5 downto 0);

-- MIPS Control Signale

signal mips_operation : std_logic_vector(5 downto 0);
signal regDst, branch, memRead, memToReg, memWrite, aluSrc, regWrite : std_logic;
signal aluOp : std_logic_vector(1 downto 0);

-- RAM Signale

signal ramWriteAddr : std_logic_vector(LOG2_NUM_RAM_ELEMENTS-1 downto 0);
signal ramWriteEn : std_logic;
signal ramIn, ram_out : std_logic_vector(RAM_ELEMENT_WIDTH-1 downto 0);
signal negativClk : std_logic;

-- Shift Signale

signal shift_out : std_logic_vector(31 downto 0);

-- Adder Signale

signal adder_withoutShift, adder_withShift : std_logic_vector(31 downto 0);
signal addFour : std_logic_vector(31 downto 0) := (others => '0');

-- ROM Signal

signal rom_out : std_logic_vector(PC_WIDTH-1 downto 0);
signal romAddr : std_logic_vector(LOG2_NUM_ROM_ELEMENTS-1 downto 0);

-- Sign Extend

signal signExtend_temp : signed(31 downto 0);
signal signExtend : std_logic_vector(31 downto 0);



begin 

-- PC

    PC_default: entity work.reg(behavioral)
        generic map(
            WIDTH => PC_WIDTH
            )

        port map(
            clk => clk,
            rst => rst,
            en => not(testMode_debug),      -- Falls testMode_debug = 1 dann en =0. Heißt Q = D wird verhindert. 
            D => pc_current,
            Q => pc_next
        );



-- bin2Chars für Testbench 

segment_tafel1: entity work.bin2Char(behavioral)
port map(
    bin => pc_next(3 downto 0),
    bitmask => pc7SegDigits_debug(0)
);

segment_tafel2: entity work.bin2Char(behavioral)
port map(
    bin => pc_next(7 downto 4),
    bitmask => pc7SegDigits_debug(1)
);

segment_tafel3: entity work.bin2Char(behavioral)
port map(
    bin => pc_next(11 downto 8),
    bitmask => pc7SegDigits_debug(2)
);

segment_tafel4: entity work.bin2Char(behavioral)
port map(
    bin => pc_next(15 downto 12),
    bitmask => pc7SegDigits_debug(3)
);


romAddr <= pc_next(11 downto 2);


-- Instruction Memory
INSTR_ROM: entity work.flashROM(behavioral)


generic map(NUM_ELEMENTS =>NUM_ROM_ELEMENTS ,
LOG2_NUM_ELEMENTS => LOG2_NUM_ROM_ELEMENTS,
ELEMENT_WIDTH => PC_WIDTH, 
INIT_FILE_NAME => PROG_FILE_NAME)


port map(address => romAddr, -- NOCHMAL SCHAUEN !!!!!
        readData => rom_out );


instruction <= testInstruction_debug when testMode_debug = '1' else rom_out;    -- Für testbench um instruction auslesen zu können

--Register

read_register1 <= instruction(25 downto 21);
read_register2 <= instruction(20 downto 16);

write_register <= instruction(15 downto 11) when regDst = '1' else instruction(20 downto 16);

Reg: entity work.regFile(structural)
        generic map( NUM_REGS => NUM_REGS,
                     LOG2_NUM_REGS => LOG2_NUM_REGS,
                     REG_WIDTH => REG_WIDTH)

        port map(clk => clk, 
                 rst => rst,
                 readAddr1 => read_register1,
                 readData1 => read_data1,
                 readAddr2 => read_register2,
                 readData2 => read_data2,
                 writeEn => regWrite,
                 writeAddr => write_register,
                 writeData => write_data,
                 reg_vect_debug => registers_debug);

-- MIPS Control
mips_operation <= instruction(31 downto 26);
MIPSctrl: entity work.mipsCtrl(structural)
            port map(op => mips_operation,
                     regDst => regDst,
                     branch => branch,
                     memRead => memRead,
                     memToReg => memToReg,
                     aluOp => aluOp,
                     memWrite => memWrite,
                     aluSrc => aluSrc,
                     regWrite => regWrite);


-- ALU 

alu_b <= signExtend when aluSrc = '1' else read_data2;

ALUmips: entity work.mipsAlu(behavioral)
            generic map(WIDTH => ALU_WIDTH
            )

            port map(ctrl => alu_control,
                     a => read_data1,
                     b => alu_b,
                     result => alu_result,
                     overflow => alu_overflow,
                     zero => alu_zero);

-- Sign Extend

    signBig: entity work.signExtend(behavioral)
            generic map(INPUT_WIDTH => 16,
                        OUTPUT_WIDTH => 32)
            
            port map(number => signed(instruction(15 downto 0)),
                     signExtNumber => signExtend_temp);
        
    signExtend <= std_logic_vector(signExtend_temp);
    
-- Alu Control

    aluControl: entity work.aluCtrl(behavioral)
                    port map(aluOp => aluOp,
                             f => instruction(5 downto 0),
                             operation => alu_control
                             );



--Shift Left 2

shiftLeft: entity work.leftShifter(behavioral)
                generic map(WIDTH => 32,
                            SHIFT_AMOUNT => 2)

                port map(number => signExtend,
                         shiftedNumber => shift_out);

-- clk invert
negativClk <= not clk;

ramWriteAddr <= ramWriteAddr_debug when ramInsertMode_debug = '1' else alu_result(11 downto 2); -- er gibt es weiter an WriteData dieser geht bis 11. Der Grund für 2 ist das 0 und 1 reserviert sind im Register (siehe Rorg VL)
ramWriteEn <= ramWriteEn_debug when ramInsertMode_debug = '1' else memWrite;
ramIn <= ramWriteData_debug when ramInsertMode_debug = '1' else read_data2;

-- Data Memory
DATA_RAM: entity work.flashRAM(behavioral)

generic map(NUM_ELEMENTS => NUM_RAM_ELEMENTS,
LOG2_NUM_ELEMENTS => LOG2_NUM_RAM_ELEMENTS,
ELEMENT_WIDTH => RAM_ELEMENT_WIDTH,
INIT_FILE_NAME => DATA_FILE_NAME)


port map(clk => negativClk,
address => ramWriteAddr,
writeEn => ramWriteEn,
writeData => ramIn,
readEn => memRead,
readData => ram_out,
ramElements_debug => ramElements_debug);

write_data <= ram_out when memToReg = '1' else alu_result;
-- Erster Adder

-- Wertigkeit 4 auf 1 setzen

addFour(2) <= '1';

AdderPlusFour: entity work.mipsAlu(behavioral)
                 generic map(WIDTH => 32)

                 port map(ctrl => "0010", -- code für addieren
                          a => pc_next,
                          b => addFour,
                          result => adder_withoutShift,
                          overflow => open,
                          zero => open);

AdderNextPC: entity work.mipsAlu(behavioral)
                generic map(WIDTH => 32)

                port map(ctrl => "0010", -- code für addieren
                         a => adder_withoutShift,
                         b => shift_out,
                         result => adder_withShift,
                         overflow => open,
                         zero => open);


-- Nächsten PC bestimmen


pc_current <= adder_withShift when (alu_zero and Branch) = '1' else adder_withoutShift;
pc_next_debug <= pc_current;




end architecture;








