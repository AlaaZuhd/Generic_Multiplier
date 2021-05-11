-----------------------------------------------------------
-- implemnation of the 2X1MUX circuit. 

-- Defining needed Libraries
library ieee;
use ieee.std_logic_1164.all; 

-- defining the entity of the 2X1MUX   
entity MUX2X1 is	   
	generic (n,m: positive); 
	port(selection: std_logic;
	    Input0, Input1: in std_logic_vector(n+m downto 0);
		output: out std_logic_vector(n+m downto 0));  
	-- the first signal "selection" is the selection line of the mux, 
	-- input0 and input1 is two signals that the mux will choose one of them based on "selection" value, each of them are m+n bit
	-- output is the result of the max which is either input0 or input1, and it's also m+n bits number. 
end entity MUX2X1;					

-- defining and building the architecture of the 2X1MUX 
architecture MUX2X1 of MUX2X1 is
signal out1, out2: std_logic_vector(n+m downto 0);
begin	
	output <= input0 when selection ='0' else input1; 	
	-- if selection = 0, then the output will take the first signal "input0", else it will take the second signal "input1". 
end architecture MUX2X1;
