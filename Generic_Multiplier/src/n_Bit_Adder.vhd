-----------------------------------------------------------
-- structural implementation of n-bit adder using generic parameter 'n' and One_Bit_Adder entity  
library ieee;
use ieee.std_logic_1164.all; 


-- defining the entity of the n-bit adder
ENTITY n_bit_adder IS
  GENERIC ( n: positive ); -- n -> generic parameter
  PORT (X, Y: IN STD_LOGIC_VECTOR(n-1 DOWNTO 0);
        Cin:  IN STD_LOGIC;
	    Sum: OUT STD_LOGIC_VECTOR(n-1 DOWNTO 0);
	    Cout: OUT STD_LOGIC); -- finish declaring the ports of the entity
		-- X and Y are the inputs to be added together. 
		-- cin is the carry in. 
		-- Sum is the register for the resulted sum.
		-- Cout is the carry out of the addition.
END ENTITY n_bit_adder;

-- internal functional of the n-bit adder 
architecture structural_N_Bit_Adder of N_Bit_Adder is 
signal Carry: std_logic_vector(n downto 0); -- carry signal to use in a the carry (in or out) for each one-bit-adder
begin	
	
	Carry(0) <= Cin; -- the first element in carry is the cin
	-- the last element in carry represents the carry out (final carry)resulted from the addition of the last bit
	Cout <= Carry(n);  
	
	-- add every two bits of the n-bit numbers X(i) and Y(i)
	-- and using the previous geenrated carry as the carry in for each current level of addition.
	generateLoop1: FOR i IN 0 TO n-1 GENERATE  
	     One_Bit_Adder_GateI:  ENTITY work.One_Bit_Adder(structural_One_Bit_Adder) PORT MAP (X(i),Y(i),Carry(i),Sum(i),Carry(i+1));
     END GENERATE generateLoop1;

end architecture structural_N_Bit_Adder;