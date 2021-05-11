
-----------------------------------------------------------
-- implemnation of a special model for the right shift, such that it's euivelent to the usuall right-shift with an additional 
-- features, such that the new left-most bit will be equal to new_left_most_bit signal.  

-- Defining needed Libraries
library ieee;
use ieee.std_logic_1164.all; 

-- defining the entity of the special_right_shift. 
entity special_right_shift is	   
	generic (K: positive); 
	port(new_left_most_bit: std_logic;
	    Input: in std_logic_vector(K-1 downto 0);
		output: out std_logic_vector(K-1 downto 0));
end entity special_right_shift;					

-- defining and building the architecture of the special_right_shift.
architecture special_right_shift of special_right_shift is
begin	
	--output(K-1) <= new_left_most_bit; -- the left most bit take the value of new_left_most_bit signal 
--	
--	-- the work of this loop is identical to output(K-2 downto 0) <= input(k-1 downto 1) 
--	generateForLoop: for i in 0 to K-2 generate 
--		begin						  
--			output(i) <= Input(i+1); 
--		end generate generateForLoop; 	
	output <= new_left_most_bit & input(k-1 downto 1); 	
	-- the final result will be identical to the following instruction in VHDL : output <= new_left_most_bit & input(k-1 downto 1) 
end architecture special_right_shift;