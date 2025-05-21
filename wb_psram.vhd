--32 bit Wishbone SRAM/PSRAM controller for 16 bit wide data device
--Copyright 2025 Quantum Design, Inc.
--
--Redistribution and use in source and binary forms, with or without modification,
-- are permitted provided that the following conditions are met:
--
--1. Redistributions of source code must retain the above copyright notice, 
-- this list of conditions and the following disclaimer.
--
--2. Redistributions in binary form must reproduce the above copyright notice, 
-- this list of conditions and the following disclaimer in the documentation 
-- and/or other materials provided with the distribution.--
--
--3. Neither the name of the copyright holder nor the names of its contributors
-- may be used to endorse or promote products derived from this software without
-- specific prior written permission.
--
--THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
--AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
--WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
--IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
--INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
--BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
--OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
--WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
--ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
--EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

--May 2025: This code could use some refactoring and cleanup, but basically seems to work
--for a simple RAM read/write test run on a host microcontroller at 8,16 & 32 bit widths, 
--(all aligned of course).
--Timing for wait states is more conservative than T_ACCESS_NS would indicate, and it
--still needs tuning and refactoring to use T_ACCESS_NS optimally. Only tested/run on Xilinx series 7
--devices clocked at 50MHz as of this writing.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

library qd_periph;

entity wb_psram_controller is
  generic (
    -- System parameters
    CLOCK_FREQUENCY : natural := 50_000_000;  -- Default: 50 MHz
    
    -- Memory mapping
    BASE_ADDRESS    : std_ulogic_vector(31 downto 0) := x"90000000"; -- Default base address in memory map
    
    -- Wishbone bus parameters
    WB_ADDR_WIDTH  : integer := 32;
    WB_DATA_WIDTH  : integer := 32;
    
    -- Pseudo-SRAM parameters
    PSRAM_ADDR_WIDTH  : integer := 19;  -- 512K addresses for IS66WV51216EBLL
    PSRAM_DATA_WIDTH  : integer := 16;  -- 16-bit data width
    
    -- Timing parameters (in nanoseconds)
    T_ACCESS_NS    : integer := 70         -- Access time (70ns for IS66WV51216EBLL-70BLI)
    -- T_REFRESH_NS   : integer := 10000   -- Refresh time (15us for IS66WV51216EBLL-70BLI)
  );
  port (
    -- System signals
    clk_i          : in  std_ulogic;
    rst_i          : in  std_ulogic;
    
    -- Wishbone slave interface
    wb_adr_i       : in  std_ulogic_vector(WB_ADDR_WIDTH-1 downto 0);
    wb_dat_i       : in  std_ulogic_vector(WB_DATA_WIDTH-1 downto 0);
    wb_dat_o       : out std_ulogic_vector(WB_DATA_WIDTH-1 downto 0);
    wb_sel_i       : in  std_ulogic_vector((WB_DATA_WIDTH/8)-1 downto 0);
    wb_cyc_i       : in  std_ulogic;
    wb_stb_i       : in  std_ulogic;
    wb_we_i        : in  std_ulogic;
    wb_ack_o       : buffer std_ulogic;
    
    -- Pseudo-SRAM interface
    psram_addr     : out std_ulogic_vector(PSRAM_ADDR_WIDTH-1 downto 0);
    psram_data_i   : in  std_ulogic_vector(PSRAM_DATA_WIDTH-1 downto 0);
    psram_data_o   : out std_ulogic_vector(PSRAM_DATA_WIDTH-1 downto 0);
    psram_data_oe  : out std_ulogic; -- Data direction of FPGA pins connected to PSRAM data bus. 1 means FPGA data out, PSRAM data in.
    psram_ce_n     : buffer std_ulogic; -- Chip enable (active low)
    psram_oe_n     : out std_ulogic; -- Output enable (active low)
    psram_we_n     : out std_ulogic; -- Write enable (active low)
    psram_ub_n     : out std_ulogic; -- Upper byte enable (active low)
    psram_lb_n     : out std_ulogic  -- Lower byte enable (active low)
    
    -- Debug 
    -- address_hit    : buffer std_ulogic
  );
end entity wb_psram_controller;

architecture rtl of wb_psram_controller is
  -- FSM states
  type state_type is (
    ST_IDLE,           -- Idle state, waiting for transaction
    ST_READ_ACCESS,    -- Reading data, waiting for access time
    ST_READ_COMPLETE,  -- Reading data, complete
    ST_WRITE_ACCESS,   -- Writing data, waiting for access time
    ST_WRITE_COMPLETE, -- Writing data, complete
    ST_NONE             -- Fall into 'other' case statement in state machine
  );
  
  -- Calculate clock cycles needed for timing parameters based on clock frequency
  -- Formula: cycles = ceiling(time_ns * frequency_hz / 1_000_000_000)
  function ns_to_cycles(time_ns : integer) return integer is
    variable result : integer;
  begin
    -- Avoid integer overflow by first dividing CLOCK_FREQUENCY by 1000
    result := time_ns * (CLOCK_FREQUENCY/1000) / 1_000_000; 
    if result < 1 then
      return 1;
    else
      return result;
    end if;
  end function;
  
  -- Timing parameters converted to clock cycles
  constant T_ACCESS       : integer := ns_to_cycles(T_ACCESS_NS);
  -- constant T_REFRESH      : integer := ns_to_cycles(T_REFRESH_NS);
  
  -- Address decoding constants
  constant ADDR_MASK      : std_ulogic_vector(31 downto 20) := (others => '1'); -- Mask for 1MB address space (20 address bits)
  
  -- State signal
  signal state        : state_type;
  
  -- Counter for access timing requirements
  signal timer        : unsigned(3 downto 0);
  
  -- Counter for refresh timing requirements:
 -- signal refresh_timer : unsigned(9 downto 0); 
  
  -- Indicates address decoding into our memopry map space here.
  signal address_hit   : std_ulogic;
   
  -- Data buffer for 16->32 bit conversion
  signal data_buffer  : std_ulogic_vector(15 downto 0);
  
  -- Byte select signal buffer: storage for wb_sel_o signals for clock cycles when wb_stb_i is not active anymore.
  signal sel_buffer   : std_ulogic_vector((WB_DATA_WIDTH/8)-1 downto 0);
  
  -- Word tracking for 32 bit multi cycle wishbone transfers accessing out 16 bit wide PSRAM.
  signal current_word : std_ulogic; -- 0 for LSW, 1 for MSW. A proxy for PSRAM address line A0.
  
  -- Mark signals for debug so they will not be optimized away.
  -- Note: MARK_DEBUG is AMD/Xilinx toolchain specific and not portable.
  attribute MARK_DEBUG : string;
  attribute MARK_DEBUG of state : signal is "TRUE";
  attribute MARK_DEBUG of timer : signal is "TRUE";
  attribute MARK_DEBUG of psram_oe_n : signal is "TRUE";
  attribute MARK_DEBUG of psram_we_n : signal is "TRUE";
  attribute MARK_DEBUG of psram_ub_n : signal is "TRUE";
  attribute MARK_DEBUG of psram_lb_n : signal is "TRUE";
  attribute MARK_DEBUG of psram_data_oe : signal is "TRUE";
  attribute MARK_DEBUG of wb_adr_i : signal is "TRUE";
  attribute MARK_DEBUG of wb_stb_i : signal is "TRUE";
  attribute MARK_DEBUG of wb_cyc_i : signal is "TRUE";
  attribute MARK_DEBUG of address_hit : signal is "TRUE";
  attribute MARK_DEBUG of psram_ce_n : signal is "TRUE";
  attribute MARK_DEBUG of psram_addr : signal is "TRUE";
  attribute MARK_DEBUG of wb_ack_o : signal is "TRUE";
  
begin
  -- Note: The neorv32 XBUS is Wishbone-B4 compliant, and is a 32 bit bus with 8 bit granularity.  This code 
  --   assumes correpondence between wb_sel_i() and wb_dat_* byte wide segments as documented
  --   in Rule 3.100 in the Wishbone-B4 architecture document.
  -- Note: The neorv32 XBUS implements Wishbone-B4 *Pipelined* read and write cycles.  This means that wb_stb_o
  --   is only asserted during the first clock cycle of each XBUS read and write transaction.  
  --   In general, the Wishbone master expects slaves to sample address, data (as appropriate) and select
  --   signals during the cycle that wb_stb_o is active.
  -- Note: Section 3.12 of the Wishbone-B4 architecture document states: When wb_cyc_i is negated, 
  --   all other master signals are *invalid*. 
  
  -- The first bus cycle when wb_stb_o is active has the PSRAM select psram_ce_n set inactive
  -- when entering ST_IDLE.  This should suffice for the refresh time needed for the PSRAM.

  -- Asynchronous assignments:
  
  -- Address decoder : Checks if the incoming address is within our 1MB memory region
  address_decoder: process(wb_adr_i, wb_stb_i, wb_cyc_i)
    variable ad_decode  : boolean;
  begin
    -- Checks if the XBUS address matches our base address (masking out the lower 20 bits)
    -- This implements a 1MB memory region starting at BASE_ADDRESS
    -- ad_decode := (wb_adr_i(31 downto 20) and ADDR_MASK(31 downto 20)) = (BASE_ADDRESS(31 downto 20) and ADDR_MASK(31 downto 20)); 
    ad_decode := (wb_adr_i(31 downto 20) = BASE_ADDRESS(31 downto 20)) ; 
    if ad_decode and wb_stb_i = '1' and wb_cyc_i = '1' then
      address_hit <= '1';
    else 
      address_hit <= '0';
    end if;  
  end process;
  
  -- Main synchronous state machine process 
  process(clk_i)    
      variable hi_word_wbsel   : std_ulogic; -- Indicates wb bus wants to transact with the high word in the PSRAM
      variable lo_word_wbsel   : std_ulogic; -- Indicates wb bus wants to transact with the low word in the PSRAM
      variable one_word_wbsel  : std_ulogic; -- Indicates wb bus wants to transact with only a single word in the PSRAM
      variable two_word_wbsel  : std_ulogic; -- Indicates wb bus wants to transact with both words in the PSRAM
      variable read_wait_ticks : unsigned(3 downto 0); 
      
    begin
    if rising_edge(clk_i) then             
      -- Process local variables (That 'update' immediately (on the *current* clock cycle))
      if wb_stb_i = '1' then -- use wishbone select signals that are valid during wb_stb_i active
        hi_word_wbsel := (wb_sel_i(2) or wb_sel_i(3)) and wb_cyc_i;
        lo_word_wbsel := (wb_sel_i(0) or wb_sel_i(1)) and wb_cyc_i;
      else  -- use the copy we stored from when wb_stb_i was active at the start of the cycle.
        hi_word_wbsel := (sel_buffer(2) or sel_buffer(3)) and wb_cyc_i;
        lo_word_wbsel := (sel_buffer(0) or sel_buffer(1)) and wb_cyc_i;
      end if;     
      one_word_wbsel := hi_word_wbsel xor lo_word_wbsel;  -- only one of the words is selected by the XBUS master
      two_word_wbsel := hi_word_wbsel and lo_word_wbsel;  -- both of the words are selected by the XBUS master
   
      -- Default values for registered signals to take on the *next* clock cycle
      psram_data_oe <= '0';          -- Default to FPGA not driving psram data bus unless overridden below.
      psram_we_n <= '1';             -- Default PSRAM write enable to inactive unless overridden below.            
      psram_addr(0) <= current_word; -- indicates high or low word of 32 bit value as LSbit of word address at PSRAM.
      if address_hit = '1' then   -- latch upper address pins to PSRAM when wb_stb_i arrives.
        psram_addr(PSRAM_ADDR_WIDTH-1 downto 1) <= wb_adr_i(PSRAM_ADDR_WIDTH downto 2); 
        psram_ce_n <= '0';  --   set PSRAM chip select active on address hit 
      end if;
        
      wb_ack_o <= '0';  -- Default is to keep bus cycle going by not 'acking' yet.

      case state is
        when ST_IDLE =>
          current_word <= '0';  -- init to lower word access.
          timer <= (others => '0');  -- clear the transaction timer
          -- Check for new Wishbone transaction that hits our address space
          if (address_hit = '1') then
            -- Address is decoded to be within this device's range and this is the first clock of the bus cycle.      
            sel_buffer <= wb_sel_i;  -- latch wb_sel while wb_stb_o is active and address decoded.
            if wb_we_i = '1' then  
              -- WRITE operation - FPGA will drive the PSRAM data bus
              state <= ST_WRITE_ACCESS;
              psram_data_oe <= '1'; -- PSRAM data bus direction : FPGA pins connected to PSRAM data become outputs
              psram_oe_n <= '1';    -- PSRAM data bus direction : PSRAM data pins become inputs 
              psram_we_n <= '0';    -- Activate PSRAM write enable
              -- Note: Data will not actually be latched by the PSRAM until psram_we_n *rises* in a later state.
              if one_word_wbsel = '1' then  -- If there is only one of the words (or byte therein) to transfer
                -- Then drive that single word data to the PSRAM, be it either the low or high word
                if lo_word_wbsel = '1' then
                  current_word <= '0';  -- start LSW read on next clock cycle
                  psram_addr(0) <= '0'; 
                  psram_lb_n <= not wb_sel_i(0);
                  psram_ub_n <= not wb_sel_i(1);
                  psram_data_o <= wb_dat_i(15 downto 0); -- Send LSW of wb data bus to PSRAM on next clock cycle
                else
                  current_word <= '1';  -- start MSW read on next clock cycle
                  psram_addr(0) <= '1';
                  psram_lb_n <= not wb_sel_i(2);
                  psram_ub_n <= not wb_sel_i(3);
                  psram_data_o <= wb_dat_i(31 downto 16); -- Send MSW of wb data bus to PSRAM on next clock cycle
                end if; --low_word_wbsel
              elsif two_word_wbsel = '1' then -- both words write requested by wb bus
                  -- Present the LSW data to the PSRAM, and do the MSW later.
                  current_word <= '0';  -- start LSW read on next clock cycle
                  psram_addr(0) <= '0';
                  psram_lb_n <= not wb_sel_i(0);
                  psram_ub_n <= not wb_sel_i(1);
                  psram_data_o <= wb_dat_i(15 downto 0); -- Send LSW of wb data bus to PSRAM on next clock cycle
                  data_buffer <= wb_dat_i(31 downto 16); -- and remember MSW for even later cycle. 
              end if;  -- one_word_websel, two_word_websel;                       
            else  --(i.e. when wb_we_i = '0')
              -- READ operation - PSRAM will drive the PSRAM data bus
              state <= ST_READ_ACCESS;
              psram_data_oe <= '0'; -- PSRAM data bus direction : FPGA pins connected to PSRAM data become inputs
              psram_oe_n <= '0';    -- PSRAM data bus direction : PSRAM data pins become outputs 
              psram_we_n <= '1';    -- Deactivate PSRAM write enable
              -- Note: wb can not latch data from PSRAM until a later cycle when read access time has elapsed.
              if one_word_wbsel = '1' then  -- If there is only one of the words (or byte therein) to transfer
                -- Then start to read that single word data from the PSRAM, be it either the low or high word
                current_word <= hi_word_wbsel;  
                psram_addr(0) <= hi_word_wbsel;
                if hi_word_wbsel = '1' then  -- Pick which word to read from PSRAM
                  psram_lb_n <= not wb_sel_i(2);
                  psram_ub_n <= not wb_sel_i(3);
                else
                  psram_lb_n <= not wb_sel_i(0);
                  psram_ub_n <= not wb_sel_i(1);
                end if;               
              elsif two_word_wbsel = '1' then -- Both words write requested by wb bus
                -- Request the LSW data to the PSRAM, and do the MSW later.
                  current_word <= '0';  -- start LSW write on next clock cycle
                  psram_addr(0) <= '0';  
                  psram_lb_n <= not wb_sel_i(0);
                  psram_ub_n <= not wb_sel_i(1);
              end if; -- if one/two_word_wbsel
            end if; -- read cycle start vs. write cycle start
          end if;  -- elsif (address_hit)
          
        when ST_READ_ACCESS =>
          -- Maintain drive of PSRAM data bus by PSRAM:
          psram_data_oe <= '0'; -- PSRAM data bus direction : FPGA pins connected to PSRAM data become inputs
          psram_oe_n <= '0';    -- PSRAM data bus direction : PSRAM data pins become outputs 
          psram_we_n <= '1';    -- Deactivate PSRAM write enable
          if two_word_wbsel = '1' and current_word = '1' then
            read_wait_ticks:= to_unsigned(T_ACCESS, timer'length); -- need an extra clock cycle for this one.
          else
            --read_wait_ticks:= to_unsigned(T_ACCESS - 1, timer'length);
            read_wait_ticks:= to_unsigned(T_ACCESS, timer'length);
          end if;                   
          if timer < read_wait_ticks then
            -- Stay in this state and spin until access time expired, to allow PSRAM data output to settle.
            timer <= timer + 1;
          else  
            timer <= (others => '0');  -- clear the transaction timer on transition to next clock cycle.
            -- Take action appropriate to the expiration of the read access timer:
            if two_word_wbsel = '1' then
              if current_word = '0' then     -- if a second word still needs transfer
                -- store the first LSW locally to hand to the wb later
                data_buffer <= psram_data_i;
                -- We choose that most significant word always comes second in a compound transfer.
                -- Stay in the same state and wait for the psram to emit the second word.
                current_word <= '1'; 
                --  psram_addr(0) <= '1'; commented out to keep old A0 valid for data_buffer to latch. 
                psram_lb_n <= not sel_buffer(2);  -- move byte selects at PSRAM to match the new MSW.
                psram_ub_n <= not sel_buffer(3);
              else
                -- Present the data_buffer stored eariler to the least significant bits of wb_dat_o 
                wb_dat_o(15 downto 0) <= data_buffer;
                -- and the top half of the 32-bit transfer is sitting at the PSRAM output now:
                wb_dat_o(31 downto 16) <= psram_data_i;
                state <= ST_READ_COMPLETE;
                wb_ack_o <= '1';  -- let wishbone know that it can latch the data on wb_dat_o now
              end if;  -- if current_word
            else  -- handles one word (or pathological zero word case)
              if current_word = '0' then  -- handles one_word_wbsel cases for driving PSRAM data bus
                wb_dat_o(15 downto 0) <= psram_data_i;  -- Maintain LSW data on next clock cycle
              else
                wb_dat_o(31 downto 16) <= psram_data_i;  -- Maintain MSW data on next clock cycle
              end if; 
              state <= ST_READ_COMPLETE;
              wb_ack_o <= '1';  -- let wishbone know that it can latch the data on wb_dat_o now
            end if; -- if two_word_wbsel else
          end if; -- if access time timer expired else 
                     
        when ST_READ_COMPLETE =>  -- wb_ack_o active during this cycle.
          state <= ST_IDLE;    -- Finish wb Cycle. Note wb_dat_o latches data at the end of this cycle (when wb_ack_o falls)
          psram_lb_n <= '1';   -- inactivate byte selects
          psram_ub_n <= '1';
          psram_ce_n <= '1';
          psram_oe_n <= '1';
    
        when ST_WRITE_ACCESS =>
          psram_data_oe <= '1'; -- PSRAM data bus direction : FPGA pins connected to PSRAM data become outputs
          psram_oe_n <= '1';    -- PSRAM data bus direction : PSRAM data pins become inputs 
          psram_we_n <= '0';    -- Activate PSRAM write enable          
          -- Wait for access time
          if timer < to_unsigned(T_ACCESS - 1, timer'length) then  -- subtract 1 because timer does not count _wb_ack active cycle.
            timer <= timer + 1;
            -- Stay in this same state for the next clock cycle.
            -- Leaving psram_data_o in the state it had been driven to in the ST_IDLE state.
          else  -- time to latch some data into the PSRAM on next clock edge:
            if one_word_wbsel = '1' then -- One 16 bit write cycle is all that is needed.
              state <= ST_WRITE_COMPLETE; -- So don't do another write to PSRAM - end the wb bus cycle next clock cycle.
              psram_ce_n <= '0';  -- keep select active during next cycle.
              wb_ack_o <= '1';  -- tell wishbone we are done and have latched the selected data.             
            elsif two_word_wbsel = '1' then -- two 16 bit write cycles at the PSRAM are needed
              if current_word = '0' then  -- Write LSW to PSRAM and set up to do the MSW in this same wb bus cycle
                -- Stay in the same state to write the MSW to the PSRAM in a subsequent cycle .
                current_word <= '1';  -- select MSW next
                psram_addr(0) <= '1';
                psram_lb_n <= not sel_buffer(2);  -- move byte selects at PSRAM to match the new MSW pattern the XBUS selected
                psram_ub_n <= not sel_buffer(3);
                psram_data_o <= data_buffer;  -- put stored MSW data from ST_IDLE state onto PSRAM data pins
                psram_we_n <= '1';  -- Deactivate PSRAM write enable for one cycle to latch first word at PSRAM.
                timer <= (others => '0');  -- reset access timer
                -- Note that for the  IS66WV51216EBLL-70BLI, we get 3 clocks = 60ns before moving on from this first write to PSRAM.
                --   This meets most timing on the datasheet, but technically violates one: the 'total' write cycle time of 70 ns.
                --   If this becomes a problem, we can similarly do here as we did in the 32 bit read using read_wait_ticks.
              else 
                psram_ce_n <= '0';  -- keep select active during next cycle.
                wb_ack_o <= '1';  -- tell wishbone we are done (we have latched both MSW and LSW data at PSRAM) 
                state <= ST_WRITE_COMPLETE;
              end if; -- if current_word
            end if; -- elsif two_word_wbsel
          end if; -- if timer
           
        when ST_WRITE_COMPLETE =>
          -- Finish write cycle. wb_ack_o should be active during this cycle.
          -- Deactivate PSRAM selects at end of this cycle to allow the ack cycle to also be a wait state.
          -- PSRAM should latch data on psram_data_o at the end of this cycle.
          state <= ST_IDLE;
          psram_lb_n <= '1';    -- Deactivate byte selects
          psram_ub_n <= '1';
          psram_ce_n <= '1';    -- Dectivate PSRAM  chip enable
          psram_we_n <= '1';    -- Dectivate PSRAM write enable 
                   
        when others =>  -- like ST_NONE :)
          state <= ST_IDLE;
          psram_lb_n <= '1';  -- inactivate byte selects
          psram_ub_n <= '1';
          psram_ce_n <= '1';
      end case; -- state
      
      -- Handle synch. reset last so it will overwrite above signal assignments when it is needed.
      
      if rst_i = '1' then  -- WB-B4 Rule 3.00: Resets need to be synchronous to clk_i.
        state <= ST_NONE;  -- Start in this state to get a clock cycle of neutral behavior after reset.
        current_word <= '0';
        timer <= (others => '0');
        data_buffer <= (others => '0');
        --refresh_timer <= (others => '0');
        psram_lb_n <= '1';  -- inactivate byte selects
        psram_ub_n <= '1';
      end if; -- rst_i
      
    end if;  -- rising_edge(clk_i)
  end process; -- (clk_i) synchronous state machine
  
end architecture rtl;

-- End of qd_periph.wb_psram_controller