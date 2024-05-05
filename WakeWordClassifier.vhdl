----------------------------------------------------------------------------
-- Project		:	Invent a Chip
-- Authors		:	Malte Hawich, Simon Klein
-- Description	:	Top level entity for the WakeWordClassifier for the Invent a Chip project
--              This is an example implementation of the WakeWordClassifier entity
--              In this task you should only learn how to communicate with the BufferBank
--              Therefore mulitple signals are needed, and you need to build your own little state machine
--              all signals are explained in comment blocks within the code.
--
--                  For problems, questions, suggestions or any other inquiries
--                  please contact inventachip@ims.uni-hannover.de 
--                  or ask in the Git Repository
----------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
-- use work.iac_pkg.all;
use work.common_pkg.all;

entity WakeWordClassifier is
  port (
    clk                 : in std_logic;
    reset               : in std_logic;
    enable              : in std_logic;
    wakeword_detected   : out std_logic;
    -- Sdram access
    sdram_select        : out std_ulogic;
    sdram_data_ack      : out std_ulogic;
    sdram_write_en      : out std_ulogic;
    sdram_address       : out std_ulogic_vector(24 downto 0);
    sdram_data_out      : out std_ulogic_vector(15 downto 0);
    sdram_data_in       : in std_ulogic_vector(15 downto 0);
    sdram_request_en    : out std_ulogic;
    sdram_req_slot_free : in std_ulogic;
    sdram_data_avail    : in std_ulogic
    -- Add your ports here
  );
end WakeWordClassifier;

architecture behavioural of WakeWordClassifier is
  -- Internal parameters declarations
  constant RAM_DEPTH  : natural := 288;
  constant DATA_WIDTH : natural := 16;
  -- Add your additional parameters here

  -- Internal component declarations
  component BufferBank is
    generic (
      BUFFER_SIZE_ONE   : natural := 16;
      BUFFER_SIZE_TWO   : natural := 16;
      BUFFER_SIZE_THREE : natural := 16
    );
    port (
      clk                 : in std_ulogic;
      reset               : in std_ulogic;
      enable              : in std_ulogic;
      -- Sdram access
      sdram_select        : out std_ulogic;
      sdram_data_ack      : out std_ulogic;
      sdram_write_en      : out std_ulogic;
      sdram_address       : out std_ulogic_vector(24 downto 0);
      sdram_data_out      : out std_ulogic_vector(15 downto 0);
      sdram_data_in       : in std_ulogic_vector(15 downto 0);
      sdram_request_en    : out std_ulogic;
      sdram_req_slot_free : in std_ulogic;
      sdram_data_avail    : in std_ulogic;
      -- Request ports
      img_buf_req_enable  : in std_ulogic;
      filt_buf_req_enable : in std_ulogic;
      request_rdy         : out std_ulogic;
      request_address     : in std_ulogic_vector(24 downto 0); -- sdram_addr_req
      buf_full            : out std_ulogic;                    -- all buffer filled
      wr_req_done         : out std_ulogic;                    -- write request done
      write_request       : in std_ulogic;                     -- request to write to sdram - result_buf_enable (always just one word)
      data_in             : in std_ulogic_vector(15 downto 0); -- just for data to sdram
      start               : out std_ulogic;                    -- start bit signals the WakeWordClassifier, that the config is loaded
      -- Data access from buffer
      data_out_img_buf    : out std_ulogic_vector(15 downto 0);
      address_img_buf     : in unsigned(8 downto 0); -- img_addr
      data_out_filt_buf   : out std_ulogic_vector(15 downto 0);
      address_filt_buf    : in unsigned(8 downto 0); -- filt_addr
      data_out_cfg_buf    : out std_ulogic_vector(15 downto 0);
      address_cfg_buf     : in unsigned(8 downto 0) -- config_addr
    );
  end component;

  -- ##############################################################################################################
  -- ### Buffer Bank                                                                                            ###
  -- ### The Buffer Bank functions as a Buffer for memory access to the slow SDRAM.                             ###
  -- ### It is used to store the image data, filter data and configuration data.                                ###
  -- ### The Buffer Bank has three buffers, one for each data type.                                             ###
  -- ### Each buffer can be accessed with its own address.                                                      ###
  -- ### The data from the SDRAM needs to be loaded into the buffer first, before it can be accessed.           ###
  -- ### This happens with the img_buf_enable, filt_buf_enable signals, aswell as the request_address           ###
  -- ### signal, which is the address in the SDRAM.                                                             ###
  -- ###                                                                                                        ###
  -- ### The Buffer Bank signals, that it is ready to receive a request by setting the request_rdy signal high  ###
  -- ### It is setup to work with a Neural Network Accelerator and therefore load the config first before       ###
  -- ### signaling that it is ready to receive a request.                                                       ###
  -- ### The Buffer Bank needs to be enabled by setting the enable signal high.                                 ###
  -- ### Then you need to wait for its ready signal to be high before sending a request.                        ###
  -- ### The Buffer Bank always expects img request first, so for your task you only need to request img data.  ###
  -- ###                                                                                                        ###
  -- ### The Buffer Bank will signal that it is done with requesting your data by setting the                   ###
  -- ### buf_full signal high.                                                                                  ###
  -- ###                                                                                                        ###
  -- ### After that you can access the data directly with the address_img_buf values, which are the addresses   ###
  -- ### of the data in the buffer.                                                                             ###
  -- ### The data then can be accessed with the data_out_img_buf signal, in the next clock cycle                ###
  -- ###                                                                                                        ###
  -- ### Writing happens dircetly through the Buffer Bank and not into it. Therfore you need to set the         ###
  -- ### write_request signal high and provide the data to be written with the data_in signal.                  ###
  -- ### The Buffer Bank will signal that it is done with writing by setting the wr_req_done signal high.       ###
  -- ### After that you can send another write reqeust or request new read data.                                ###
  -- ##############################################################################################################

  -- internal signals declarations
  type state_type is (not_enabled, startup, request_data, get_data, write_back, finished);
  signal state, state_next            : state_type;
  -- buffer bank communication
  signal buf_bank_img_buf_req_enable  : std_ulogic;
  signal buf_bank_filt_buf_req_enable : std_ulogic;
  signal buf_bank_request_address     : std_ulogic_vector(24 downto 0);
  signal buf_bank_buf_full            : std_ulogic;
  signal buf_bank_write_request       : std_ulogic;
  signal buf_bank_data_in             : std_ulogic_vector(15 downto 0);
  signal buf_bank_start               : std_ulogic;
  signal buf_bank_data_out_img_buf    : std_ulogic_vector(15 downto 0);
  signal buf_bank_address_img_buf     : unsigned(8 downto 0);
  signal buf_bank_data_out_filt_buf   : std_ulogic_vector(15 downto 0);
  signal buf_bank_address_filt_buf    : unsigned(8 downto 0);
  signal buf_bank_data_out_cfg_buf    : std_ulogic_vector(15 downto 0);
  signal buf_bank_address_cfg_buf     : unsigned(8 downto 0);
  signal buf_bank_wr_req_done         : std_ulogic;
  signal request_rdy_fsm              : std_ulogic;
  
  
  -- Add your internsdramal signals here
  signal sdram_req_adress : std_ulogic_vector (24 downto 0);
  signal sdram_req_adress_next : std_ulogic_vector (24 downto 0);

begin
  -- Add your component instantiations here
  buf_bank : BufferBank
  generic map(
    BUFFER_SIZE_ONE   => 16,
    BUFFER_SIZE_TWO   => 16,
    BUFFER_SIZE_THREE => 16
  )
  port map(
    clk                 => clk,
    reset               => reset,
    enable              => enable,
    -- Sdram access
    sdram_select        => sdram_select,
    sdram_data_ack      => sdram_data_ack,
    sdram_write_en      => sdram_write_en,
    sdram_address       => sdram_address,
    sdram_data_out      => sdram_data_out,
    sdram_data_in       => sdram_data_in,
    sdram_request_en    => sdram_request_en,
    sdram_req_slot_free => sdram_req_slot_free,
    sdram_data_avail    => sdram_data_avail,
    -- Request ports
    img_buf_req_enable  => buf_bank_img_buf_req_enable,
    filt_buf_req_enable => buf_bank_filt_buf_req_enable,
    request_rdy         => request_rdy_fsm,
    request_address     => buf_bank_request_address,
    buf_full            => buf_bank_buf_full,
    wr_req_done         => buf_bank_wr_req_done,
    write_request       => buf_bank_write_request,
    data_in             => buf_bank_data_in,
    start               => buf_bank_start,
    -- Data access from buffer
    data_out_img_buf    => buf_bank_data_out_img_buf,
    address_img_buf     => buf_bank_address_img_buf,
    data_out_filt_buf   => buf_bank_data_out_filt_buf,
    address_filt_buf    => buf_bank_address_filt_buf,
    data_out_cfg_buf    => buf_bank_data_out_cfg_buf,
    address_cfg_buf     => buf_bank_address_cfg_buf
  );

  -- Add your code for the WakeWordClassifier here


  seq : process(clk, reset)
  begin 
    if reset = '1' then 
      state <= not_enabled;
      sdram_req_adress <= (others => '0');
    elsif rising_edge(clk) then 
      if enable = '1' then
      state <= state_next;
      sdram_req_adress <= sdram_req_adress_next;
      else 
      state <= state;
      sdram_req_adress <= sdram_req_adress;
      end if;
    end if;
  end process seq; 

  main : process (state)
  begin 
  state_next <= state;
    sdram_req_adress_next <= sdram_req_adress;
    buf_bank_request_address <= (others => '0');
    buf_bank_img_buf_req_enable <= '0';
    buf_bank_write_request <= '0';
    wakeword_detected <= '0';
    buf_bank_data_in <= (others => '0');
    buf_bank_address_img_buf <= (others => '0');
     case state is 
      when not_enabled => 
      state_next <= startup;
      when startup =>
        if request_rdy_fsm = '1' then
          state_next <= request_data;
        end if;
      when request_data => 
       
        if unsigned(sdram_req_adress) <= 5 then 
        buf_bank_request_address <= sdram_req_adress;
          buf_bank_img_buf_req_enable <= '1';
          sdram_req_adress_next <= std_ulogic_vector(unsigned(sdram_req_adress)+1);
        else
        state_next <= get_data;
        end if; 
      when get_data =>
        if buf_bank_buf_full = '1' then
          state_next <= write_back;
          sdram_req_adress_next <= std_ulogic_vector(to_unsigned(1, sdram_req_adress_next'length));
          buf_bank_address_img_buf <= (others => '0');
        end if; 
      when write_back =>
        if unsigned(sdram_req_adress) <= 6 then
          buf_bank_write_request <= '1';
          buf_bank_request_address <= std_ulogic_vector(unsigned(sdram_req_adress)+30 -1); --worjs on another counter, one
          sdram_req_adress_next <= std_ulogic_vector(unsigned(sdram_req_adress)+1);
          buf_bank_address_img_buf <= unsigned(sdram_req_adress(8 downto 0));
          buf_bank_data_in <= std_ulogic_vector(unsigned(buf_bank_data_out_img_buf)+19);
        else
          state_next <= finished;
        end if;
      when finished =>
        state_next <= not_enabled;
        wakeword_detected <= '1';
        when others =>
    end case;
  end process main;
end behavioural;