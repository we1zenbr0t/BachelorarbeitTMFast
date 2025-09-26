use ieee.numeric_std.all;

architecture TFL_FAST_DOSING_APPLICATION of TFL_FAST_USER_e is

    -- Total number of user-controlled channels
    constant CHANNEL_COUNT : natural := 8;

    -- Channel mapping arrays to assign input/output indices per user/customer logic configuration
    type t_channel_map is array (0 to 13) of integer;
    constant CHANNEL_MAP_IN_CH  : t_channel_map := (0, 1, 3, 4, 6, 7, 9, 10, 0, 1, 2, 3, 4, 5);
    constant CHANNEL_MAP_OUT_CH : t_channel_map := (0, 1, 3, 4, 6, 7, 9, 10, 2, 5, 8, 11, 6, 7);

    -- Array types for channel-wise signal/data storage
    type t_enc_count_array is array (0 to CHANNEL_COUNT-1) of std_logic_vector(31 downto 0); -- 32-bit encoder value per channel
    type t_std_logic_array   is array (0 to CHANNEL_COUNT-1) of std_logic;
    type t_integer_array     is array (0 to CHANNEL_COUNT-1) of integer range 0 to 2;

    -- Per-channel encoder/counter signals and state flags
    signal enc_count, upper_Limit, next_upper_limit, FBIF, frequency_speed : t_enc_count_array := (others => (others => '0'));
    signal home, homed, valid, underflow, overflow, lastcountdir : t_std_logic_array := (others => '0');
    signal LimitReached      : t_std_logic_array := (others => '0'); -- Set to '1' if limit reached for channel
    signal load_delay_cnt    : t_integer_array := (others => 0);     -- FSM state variable for per-channel management
    signal system_fault      : t_std_logic_array := (others => '0'); -- Per-channel fault detection logic
    signal StartDQ_active    : t_std_logic_array := (others => '0'); -- Flag: output period enabled for channel
    signal input_selector    : t_std_logic_array := (others => '0'); -- Selects digital input or RS485 for channel
	 signal WR_REC_Control    : std_logic_vector(31 downto 0); --sets Logic for all Channels

    -- Channel-wide control and control interface signals
    signal CounterReset      : t_std_logic_array := (others => '0'); -- Manual/software reset for channel
    signal polarity          : t_std_logic_array := (others => '0'); -- Output polarity for DQ/RS485
    signal EnableDQ          : t_std_logic_array := (others => '0'); -- Enable output for channel
    signal Load              : t_std_logic_array := (others => '0'); -- Triggers upper limit value buffer transfer
    signal HoldSW            : t_std_logic_array := (others => '0'); -- Hold signal for freezing counter
    signal StartDQ           : t_std_logic_array := (others => '0');
    -- StartDQ: A user or PLC pulse which clears the LimitReached latch,
    -- enabling a new counting/output cycle after an upper limit event.
    -- Output remains in the "active" (high) state until StartDQ is pulsed, for safe/cyclic operation.
    signal FBIF_Control      : t_std_logic_array := (others => '0'); -- Selects which feedback information to use

    -- Component declarations omitted for brevity
    component Inc_Enc_e
        generic (
            BIT_WIDTH : natural := 32
        );
        port (
            CLK, RST, STOP, CLKEN              : in  std_logic;
            A, B, N                            : in  std_logic;
            A_PULSE_POL, B_DIR_POL, N_ZERO_POL : in  std_logic;
            PULSECLEAR, LOAD, EDGE             : in  std_logic;
            MISSINGSUPPLY                      : in  std_logic;
            COUNT_MODE                         : in  std_logic_vector(1 downto 0);
            COUNT_TYPE                         : in  std_logic_vector(1 downto 0);
            COUNT_DIR                          : in  std_logic;
            HOLDSW, HOLDHW                     : in  std_logic;
            HOLDSOURCE, RESETSOURCE            : in  std_logic_vector(2 downto 0);
            RESETSW                            : in  std_logic;
            MAX_VALUE, MIN_VALUE               : in  std_logic_vector(31 downto 0);
            RESET_VALUE, LOAD_VALUE            : in  std_logic_vector(31 downto 0);
            ENCODERCOUNT                       : out std_logic_vector(31 downto 0);
            HOME, HOMED, UNDERFLOW, OVERFLOW   : out std_logic;
            LASTCOUNTDIR                       : out std_logic
        );
    end component;

    component FB82_Freq32_e
	 
        port (
            CLK, RST, STOP, CLKEN : in  std_logic;
            EN                    : in  std_logic;
            IN1                   : in  std_logic;
            VALID                 : out std_logic;
            PERIOD                : in  std_logic_vector(31 downto 0);
            OUT1                  : out std_logic_vector(31 downto 0)
        );
    end component;

begin

--------------------------------------------------------------------------------------------------------------------
-- Control and Feedback Interface Mapping Process
-- This process transfers CTRL_IFx bits to internal signals per channel, and may map feedback words accordingly.
--------------------------------------------------------------------------------------------------------------------
ctrl_fb_mapping_proc : process(CLK, RST)
begin
    -- Asynchronous reset for all interface and control signals
    if RST = '1' then
        FB_IF0 <= (others => '0');
        FB_IF1 <= (others => '0');
        FB_IF2 <= (others => '0');
        FB_IF3 <= (others => '0');
        CounterReset  <= (others => '0');
        polarity      <= (others => '0');
        EnableDQ      <= (others => '0');
        Load          <= (others => '0');
        HoldSW        <= (others => '0');
        StartDQ       <= (others => '0');
        FBIF_Control  <= (others => '0');
    elsif rising_edge(CLK) then
        for i in 0 to CHANNEL_COUNT-1 loop
		  
				if CPU_STOP = '1' then
					if WR_REC_Control = x"00000000" then
						 -- Abort mode: Immediately switch outputs off, reset all state
						 CounterReset(i) <= '1';
						 polarity(i) <= '0';
						 EnableDQ(i) <= '0';
						 Load(i) <= '0';
						 HoldSW(i) <= '0';
						 StartDQ(i) <= '0';
						 FBIF_Control(i) <= '0';
					else
						-- Continue mode: Group continues working as if nothing happened
						 if StartDQ_active(i) = '1' then
							EnableDQ(i) <= '1'; 
						 end if;
					end if;
				 
				else
					 -- Map user/PLC/PLC-IO interface control bits for each channel
						if i = 0 then
								  -- Control Interface Mapping
								  CounterReset(i)<= CTRL_IF0(24);
								  polarity(i)    <= CTRL_IF0(25);
								  EnableDQ(i)    <= CTRL_IF0(26);
								  Load(i)        <= CTRL_IF0(27);
								  HoldSW(i)      <= CTRL_IF0(28);
								  StartDQ(i)     <= CTRL_IF0(29);
								  FBIF_Control(i)<= CTRL_IF0(30);
								  
								  -- Feedback Interface Mapping
								  FB_IF0(29 downto 16) <= FBIF(i)(13 downto 0);
								  FB_IF0(31) <= StartDQ_active(i);
								  FB_IF0(30) <= FBIF_Control(i);

							 elsif i = 1 then
								  -- Control Interface Mapping
								  CounterReset(i)<= CTRL_IF0(8);
								  polarity(i)    <= CTRL_IF0(9);
								  EnableDQ(i)    <= CTRL_IF0(10);
								  Load(i)        <= CTRL_IF0(11);
								  HoldSW(i)      <= CTRL_IF0(12);
								  StartDQ(i)     <= CTRL_IF0(13);
								  FBIF_Control(i)<= CTRL_IF0(14);
								  
								  -- Feedback Interface Mapping
								  FB_IF0(13 downto 0) <= FBIF(i)(13 downto 0);
								  FB_IF0(15) <= StartDQ_active(i);
								  FB_IF0(14) <= FBIF_Control(i);

							 elsif i = 2 then
								  
								  -- Control Interface Mapping
								  CounterReset(i)<= CTRL_IF1(24);
								  polarity(i)    <= CTRL_IF1(25);
								  EnableDQ(i)    <= CTRL_IF1(26);
								  Load(i)        <= CTRL_IF1(27);
								  HoldSW(i)      <= CTRL_IF1(28);
								  StartDQ(i)     <= CTRL_IF1(29);
								  FBIF_Control(i)<= CTRL_IF1(30);
								  
								  -- Feedback Interface Mapping
								  FB_IF1(29 downto 16) <= FBIF(i)(13 downto 0);
								  FB_IF1(31) <= StartDQ_active(i);
								  FB_IF1(30) <= FBIF_Control(i);

							 elsif i = 3 then
								  -- Control Interface Mapping
								  CounterReset(i)<= CTRL_IF1(8);
								  polarity(i)    <= CTRL_IF1(9);
								  EnableDQ(i)    <= CTRL_IF1(10);
								  Load(i)        <= CTRL_IF1(11);
								  HoldSW(i)      <= CTRL_IF1(12);
								  StartDQ(i)     <= CTRL_IF1(13);
								  FBIF_Control(i)<= CTRL_IF1(14);
								  
								  -- Feedback Interface Mapping
								  FB_IF1(13 downto 0) <= FBIF(i)(13 downto 0);
								  FB_IF1(15) <= StartDQ_active(i);
								  FB_IF1(14) <= FBIF_Control(i);
								  
							 elsif i = 4 then
								 -- Control Interface Mapping
								 CounterReset(i)<= CTRL_IF2(24);
								 polarity(i)    <= CTRL_IF2(25);
								 EnableDQ(i)    <= CTRL_IF2(26);
								 Load(i)        <= CTRL_IF2(27);
								 HoldSW(i)      <= CTRL_IF2(28);
								 StartDQ(i)     <= CTRL_IF2(29);
								 FBIF_Control(i)<= CTRL_IF2(30);
								 
								 -- Feedback Interface Mapping
								 FB_IF2(29 downto 16) <= FBIF(i)(13 downto 0);
								 FB_IF2(31) <= StartDQ_active(i);
								 FB_IF2(30) <= FBIF_Control(i);

							elsif i = 5 then
								 -- Control Interface Mapping
								 CounterReset(i)<= CTRL_IF2(8);
								 polarity(i)    <= CTRL_IF2(9);
								 EnableDQ(i)    <= CTRL_IF2(10);
								 Load(i)        <= CTRL_IF2(11);
								 HoldSW(i)      <= CTRL_IF2(12);
								 StartDQ(i)     <= CTRL_IF2(13);
								 FBIF_Control(i)<= CTRL_IF2(14);
								 
								 -- Feedback Interface Mapping
								 FB_IF2(13 downto 0) <= FBIF(i)(13 downto 0);
								 FB_IF2(15) <= StartDQ_active(i);
								 FB_IF2(14) <= FBIF_Control(i);

							elsif i = 6 then
								 -- Control Interface Mapping
								 CounterReset(i)<= CTRL_IF3(24);
								 polarity(i)    <= CTRL_IF3(25);
								 EnableDQ(i)    <= CTRL_IF3(26);
								 Load(i)        <= CTRL_IF3(27);
								 HoldSW(i)      <= CTRL_IF3(28);
								 StartDQ(i)     <= CTRL_IF3(29);
								 FBIF_Control(i)<= CTRL_IF3(30);
								 
								 -- Feedback Interface Mapping
								 FB_IF3(29 downto 16) <= FBIF(i)(13 downto 0);
								 FB_IF3(31) <= StartDQ_active(i);
								 FB_IF3(30) <= FBIF_Control(i);

							elsif i = 7 then
								 -- Control Interface Mapping
								 CounterReset(i)<= CTRL_IF3(8);
								 polarity(i)    <= CTRL_IF3(9);
								 EnableDQ(i)    <= CTRL_IF3(10);
								 Load(i)        <= CTRL_IF3(11);
								 HoldSW(i)      <= CTRL_IF3(12);
								 StartDQ(i)     <= CTRL_IF3(13);
								 FBIF_Control(i)<= CTRL_IF3(14);
								 
								 -- Feedback Interface Mapping
								 FB_IF3(13 downto 0) <= FBIF(i)(13 downto 0);
								 FB_IF3(15) <= StartDQ_active(i);
								 FB_IF3(14) <= FBIF_Control(i);
							elsif i = 8 then  -- CH0 -> DIQ2
								 -- Control Interface Mapping
								 CounterReset(i)<= CTRL_IF4(24);
								 polarity(i)    <= CTRL_IF4(25);
								 EnableDQ(i)    <= CTRL_IF4(26);
								 Load(i)        <= CTRL_IF4(27);
								 HoldSW(i)      <= CTRL_IF4(28);
								 StartDQ(i)     <= CTRL_IF4(29);
								 FBIF_Control(i)<= CTRL_IF4(30);
								 
								 -- Feedback Interface Mapping
								 FB_IF4(29 downto 16) <= FBIF(i)(13 downto 0);
								 FB_IF4(31) <= StartDQ_active(i);
								 FB_IF4(30) <= FBIF_Control(i);

							elsif i = 9 then  -- CH1 -> DIQ5
								 -- Control Interface Mapping
								 CounterReset(i)<= CTRL_IF4(8);
								 polarity(i)    <= CTRL_IF4(9);
								 EnableDQ(i)    <= CTRL_IF4(10);
								 Load(i)        <= CTRL_IF4(11);
								 HoldSW(i)      <= CTRL_IF4(12);
								 StartDQ(i)     <= CTRL_IF4(13);
								 FBIF_Control(i)<= CTRL_IF4(14);
								 
								 -- Feedback Interface Mapping
								 FB_IF4(13 downto 0) <= FBIF(i)(13 downto 0);
								 FB_IF4(15) <= StartDQ_active(i);
								 FB_IF4(14) <= FBIF_Control(i);
								 
							elsif i = 10 then  -- CH2 -> DIQ8
								 -- Control Interface Mapping
								 CounterReset(i)<= CTRL_IF5(24);
								 polarity(i)    <= CTRL_IF5(25);
								 EnableDQ(i)    <= CTRL_IF5(26);
								 Load(i)        <= CTRL_IF5(27);
								 HoldSW(i)      <= CTRL_IF5(28);
								 StartDQ(i)     <= CTRL_IF5(29);
								 FBIF_Control(i)<= CTRL_IF5(30);
								 
								 -- Feedback Interface Mapping
								 FB_IF5(29 downto 16) <= FBIF(i)(13 downto 0);
								 FB_IF5(31) <= StartDQ_active(i);
								 FB_IF5(30) <= FBIF_Control(i);
								 
							elsif i = 11 then  -- CH3 -> DIQ11
								 -- Control Interface Mapping
								 CounterReset(i)<= CTRL_IF5(8);
								 polarity(i)    <= CTRL_IF5(9);
								 EnableDQ(i)    <= CTRL_IF5(10);
								 Load(i)        <= CTRL_IF5(11);
								 HoldSW(i)      <= CTRL_IF5(12);
								 StartDQ(i)     <= CTRL_IF5(13);
								 FBIF_Control(i)<= CTRL_IF5(14);
								 
								 -- Feedback Interface Mapping
								 FB_IF5(13 downto 0) <= FBIF(i)(13 downto 0);
								 FB_IF5(15) <= StartDQ_active(i);
								 FB_IF5(14) <= FBIF_Control(i);
								 
							elsif i = 12 then  -- CH4 -> CH6
								 -- Control Interface Mapping
								 CounterReset(i)<= CTRL_IF6(24);
								 polarity(i)    <= CTRL_IF6(25);
								 EnableDQ(i)    <= CTRL_IF6(26);
								 Load(i)        <= CTRL_IF6(27);
								 HoldSW(i)      <= CTRL_IF6(28);
								 StartDQ(i)     <= CTRL_IF6(29);
								 FBIF_Control(i)<= CTRL_IF6(30);
								 
								 -- Feedback Interface Mapping
								 FB_IF6(29 downto 16) <= FBIF(i)(13 downto 0);
								 FB_IF6(31) <= StartDQ_active(i);
								 FB_IF6(30) <= FBIF_Control(i);

							elsif i = 13 then  -- CH5 -> CH7
								 -- Control Interface Mapping
								 CounterReset(i)<= CTRL_IF6(8);
								 polarity(i)    <= CTRL_IF6(9);
								 EnableDQ(i)    <= CTRL_IF6(10);
								 Load(i)        <= CTRL_IF6(11);
								 HoldSW(i)      <= CTRL_IF6(12);
								 StartDQ(i)     <= CTRL_IF6(13);
								 FBIF_Control(i)<= CTRL_IF6(14);
			 
								 -- Feedback Interface Mapping
								 FB_IF6(13 downto 0) <=  FBIF(i)(13 downto 0);
								 FB_IF6(15) <= StartDQ_active(i);
								 FB_IF6(14) <= FBIF_Control(i);

							 else
								  CounterReset(i) <= '0';
								  polarity(i)    <= '0';
								  EnableDQ(i)    <= '0';
								  Load(i)        <= '0';
								  HoldSW(i)      <= '0';
								  StartDQ(i)     <= '0';
								  FBIF_Control(i)<= '0';
							 end if;
							 --Errors					 
							 if system_fault(i) = '1' then
								 -- Kanalindex für alle Kanäle
								 FB_IF7(27 downto 24) <= std_logic_vector(to_unsigned(i+1, 4));
								 FB_IF7(11) <= LP_QI_BAD;  -- Spannungsversorgung gilt für alle
								 FB_IF7(7) <= system_fault(i); 
								 FB_IF7(3) <= '0'; --to see if the application is aktiv in TIA

								 -- Unterscheidung nach Kanaltyp
								 if i < 8 then  
									  -- Original DI/DQ Kanäle (0-7)
									  FB_IF7(23) <= DI_QI_BAD(CHANNEL_MAP_IN_CH(i));
									  FB_IF7(19) <= DQ_QI_BAD(CHANNEL_MAP_OUT_CH(i));
									  FB_IF7(15) <= '0';  -- Kein RS485 für diese Kanäle
								 elsif i < 12 then  
									  -- CH0-CH3 als DIQ (Kanäle 8-11)
									  FB_IF7(23) <= DI_QI_BAD(CHANNEL_MAP_IN_CH(i));
									  FB_IF7(19) <= DQ_QI_BAD(CHANNEL_MAP_OUT_CH(i));
									  FB_IF7(15) <= RS485_QI_BAD(CHANNEL_MAP_IN_CH(i));
								 else  
									  -- CH4-CH7 als RS485 (Kanäle 12-13)
									  FB_IF7(23) <= '0';  -- Kein DI Fehler
									  FB_IF7(19) <= '0';  -- Kein DQ Fehler
									  FB_IF7(15) <= RS485_QI_BAD(CHANNEL_MAP_IN_CH(i)) or RS485_QI_BAD(CHANNEL_MAP_OUT_CH(i));
								 end if;
							else -- Wenn system_fault(i) '0' ist, müssen die Fehlerdetails explizit gelöscht werden
								 FB_IF7(27 downto 24) <= (others => '0'); -- Kanalindex löschen
								 FB_IF7(11) <= '0'; -- LP_QI_BAD Feedback löschen
								 FB_IF7(23) <= '0'; -- DI_QI_BAD Feedback löschen
								 FB_IF7(19) <= '0'; -- DQ_QI_BAD Feedback löschen
								 FB_IF7(15) <= '0'; -- RS485_QI_BAD Feedback löschen
								 FB_IF7(7)  <= '0';
								 FB_IF7(3)  <= '0';
						end if;
            end if;
			WR_REC_Control <= WR_REC14;
			FB_IF7(3) <= '1'; --to see if the application is aktiv in TIA
        end loop;
		      
	 end if;
end process;

-----------------------------------------------------------------------
-- Generate Block: Generate separate logic for each user logic channel
-----------------------------------------------------------------------
channel_gen : for i in 0 to CHANNEL_COUNT-1 generate

    ------------------------------------------------------------------------------
    -- Encoder Load/Limit Process for each channel
    -- This handles writing the upper limit, managing the state machine for limits,
    -- and clearing/releasing the output latch.
    ------------------------------------------------------------------------------
	 
    process(CLK, RST)
    begin
        -- Asynchronous reset: clear all logic
        if RST = '1' then
            upper_Limit(i)      <= (others => '0');
            next_upper_limit(i) <= (others => '0');
            load_delay_cnt(i)   <= 0;
            LimitReached(i)     <= '0';
        elsif rising_edge(CLK) then
            -- Latch upper limit value from the configured WR_REC on trigger
            if WR_REC_NEW = '1' then
                    if i = 0 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC00) - 1);
						 elsif i = 1 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC01) - 1);
						 elsif i = 2 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC02) - 1);
						 elsif i = 3 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC03) - 1);
						 elsif i = 4 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC04) - 1);
						 elsif i = 5 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC05) - 1);
						 elsif i = 6 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC06) - 1);
						 elsif i = 7 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC07) - 1);
						 elsif i = 8 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC08) - 1);
						 elsif i = 9 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC09) - 1);
						 elsif i = 10 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC10) - 1);
						 elsif i = 11 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC11) - 1);
						 elsif i = 12 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC12) - 1);
						 elsif i = 13 then
							  next_upper_limit(i) <= std_logic_vector(unsigned(WR_REC13) - 1);
						 end if;
					end if;
					 -- Load current encoder value to FBIF
                if FBIF_Control(i) = '1' then
                    FBIF(i) <= frequency_speed(i);
                else
                    FBIF(i) <= enc_count(i);
                end if;

            -- Limit load FSM for transfer & limit detection
            case load_delay_cnt(i) is
                when 0 =>
                    if Load(i) = '1' then
                        upper_Limit(i)    <= next_upper_limit(i);
                        load_delay_cnt(i) <= 1;
                        -- Check if limit is immediately reached
                        if unsigned(enc_count(i)) >= unsigned(next_upper_limit(i)) then
                            LimitReached(i) <= '1';
                        else
                            LimitReached(i) <= '0';
                        end if;
                    end if;
                when 1 =>
                    if Load(i) = '0' then
                        load_delay_cnt(i) <= 0;
                    end if;
                when others =>
                    load_delay_cnt(i) <= 0;
            end case;

            -- StartDQ resets the LimitReached latch to restart output cycle
            if StartDQ(i) = '1' then
                LimitReached(i) <= '0';
            end if;

            -- If overflow is detected, force LimitReached
            if overflow(i) = '1' then
                LimitReached(i) <= '1';
            end if;
				if CPU_STOP = '1' and WR_REC_Control = x"00000000" then
						 -- Abort mode: Immediately switch outputs off, reset all state
						 upper_Limit(i)      <= (others => '0');
						next_upper_limit(i) <= (others => '0');
						load_delay_cnt(i)   <= 0;
						LimitReached(i)     <= '0';
					else
						 -- Continue mode: Group continues working as if nothing happened
						 -- (Do nothing here; state machine and outputs run on)
					end if;
		  end if;
    end process;


    -- Select input source: wired digital input or RS485 depending on mapping
    input_selector(i) <= RS485_RX(CHANNEL_MAP_IN_CH(i)) when i > 8 else DI(CHANNEL_MAP_IN_CH(i));

    ----------------------------------------------------------------------
    -- Encoder Counter Instance (per channel)
    -- Standard Siemens 32-bit encoder, operates with input_selector,
    -- uses limiter, reset, hold and other signals as mapped above.
    ----------------------------------------------------------------------
    ENCODER_COUNTER : Inc_Enc_e
        generic map (
            BIT_WIDTH => 32
        )
        port map (
           	 CLK            => CLK,
				 RST            => RST,
				 A              => input_selector(i),    -- Pulse input for the corresponding channel
				 PULSECLEAR     => CounterReset(i),      -- Channel-specific reset
				 LOAD           => Load(i),              -- Channel-specific load for limit/value
				 MISSINGSUPPLY  => LP_QI_BAD,            -- Power supply monitoring/fault
				 HOLDSW         => HoldSW(i),            -- Channel-specific software hold (freeze counting)
				 MAX_VALUE      => upper_Limit(i),       -- Channel-specific maximum counter value (limit)
				 MIN_VALUE      => (others => '0'),      -- Minimum value always 0
				 RESET_VALUE    => (others => '0'),      -- Reset value always 0
				 LOAD_VALUE     => (others => '0'),      -- Load value always 0 (not used here)
				 ENCODERCOUNT   => enc_count(i),         -- Channel-specific encoder count output
				 OVERFLOW       => overflow(i),          -- Channel-specific overflow flag
				 STOP           => '0',                  -- Not used, no external stop
				 CLKEN          => '1',                  -- Always enabled, encoder always running
				 B              => '0',                  -- Not used, only pulse/direction mode
				 N              => '0',                  -- No index pulse used
				 A_PULSE_POL    => '0',                  -- No pulse polarity inversion
				 B_DIR_POL      => '0',                  -- No direction polarity inversion
				 N_ZERO_POL     => '0',                  -- No index (zero) polarity inversion
				 EDGE           => '0',                  -- Reset is dominant with simultaneous hold/reset
				 COUNT_MODE     => "11",                 -- "11" = Pulse/direction mode
				 COUNT_TYPE     => "00",                 -- "00" = Continuous counting mode
				 COUNT_DIR      => '0',                  -- No direction reversal
				 HOLDHW         => '0',                  -- No hardware hold required
				 HOLDSOURCE     => "010",                -- "010" = Only software hold active
				 RESETSW        => '0',                  -- No separate software reset, controlled via PULSECLEAR
				 RESETSOURCE    => "000"                 -- "000" = No additional reset sources active
        );

    ----------------------------------------------------------------------
    -- Frequency Measurement Instance (per channel)
    -- Optional block to provide per-channel frequency feedback if required.
    ----------------------------------------------------------------------
    FREQ_COUNTER : FB82_Freq32_e
        port map (
            CLK     => CLK,
            RST     => RST,
            STOP    => '0',
            CLKEN   => '1',
            EN      => FBIF_Control(i),
            IN1     => input_selector(i),
            VALID   => valid(i),
            PERIOD  => std_logic_vector(to_unsigned(1_000_000, 32)),
            OUT1    => frequency_speed(i)
        );
end generate;

--------------------------------------------------------------------------------
-- Fault Detection and Output Process
-- Evaluates per-channel system fault states and sets DQ/RS485 outputs accordingly.
-- Ensures safe state on error, and handles output switching logic per definition.
--------------------------------------------------------------------------------
fault_dq_proc : process(CLK, RST)
variable out_val : std_logic;
begin
    if RST = '1' then
        system_fault   <= (others => '0');
        StartDQ_active <= (others => '0');
        DQ             <= (others => '0');
    elsif rising_edge(CLK) then
        for i in 0 to CHANNEL_COUNT-1 loop
		  
				if CPU_STOP = '1' and WR_REC_Control = x"00000000" then 
					-- Abort-Fall: explizit auf SAFE/0 zurücksetzen
					DQ(i)           <= '0';
					StartDQ_active <= (others => '0');
				 else
					-- Evaluate fault state for each channel (DI/DQ/RS485x mappings and LP_QI_BAD)
            if i < 8 then
                system_fault(i) <= DI_QI_BAD(CHANNEL_MAP_IN_CH(i)) or DQ_QI_BAD(CHANNEL_MAP_OUT_CH(i)) or LP_QI_BAD;
            elsif i < 12 then
                system_fault(i) <= RS485_QI_BAD(CHANNEL_MAP_IN_CH(i)) or DQ_QI_BAD(CHANNEL_MAP_OUT_CH(i)) or LP_QI_BAD;
            else
                system_fault(i) <= RS485_QI_BAD(CHANNEL_MAP_IN_CH(i)) or RS485_QI_BAD(CHANNEL_MAP_OUT_CH(i)) or LP_QI_BAD;
            end if;
            
            -- Latch StartDQ_active for output enable
            if StartDQ(i) = '1' and LimitReached(i) = '0' then
                StartDQ_active(i) <= '1';
            elsif LimitReached(i) = '1' then
                StartDQ_active(i) <= '0';
            end if;

            -- Output logic: determine desired output state (SAFE or ACTIVE)

				if system_fault(i) = '1' then
					 -- Fault: force SAFE state depending on polarity
					 out_val := polarity(i);

				elsif EnableDQ(i) = '1' and StartDQ_active(i) = '1' then
					 -- Normal operation: ACTIVE state depending on polarity
					 out_val :=  not polarity(i);

				else
					 -- Inactive: SAFE state depending on polarity
					 out_val := polarity(i);
				end if;

				-- Drive the actual output depending on channel index
				if i < 12 then
					 DQ(CHANNEL_MAP_OUT_CH(i)) <= out_val;
				elsif i = 12 then
					 RS485_TX(6) <= out_val;
				elsif i = 13 then
					 RS485_TX(7) <= out_val;
				end if;

				 
            end if;
        end loop;
end if;
end process;
end architecture;