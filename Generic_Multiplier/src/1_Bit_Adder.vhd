
-----------------------------------------------------------
-- Structural implementation of the one bit adder.

-- Defining needed Libraries
library ieee;
use ieee.std_logic_1164.all; 

-- defining the entity of the 1-bit adder
entity One_Bit_Adder is
	port(In0, In1, Cin: in std_logic;
		Sum, Cout: out std_logic);
	--	in0 and in1 are the inputs to be added, and Cin is the carry in. 
	-- sum is the container of the addition result, and cout is for the carry out of the addition. 
end entity One_Bit_Adder;			 


-- defining and building the architecture of the 1-bit adder
architecture structural_One_Bit_Adder of One_Bit_Adder is 
signal s0, s1, s2: std_logic;
begin	
	
	Xor_In0_In1: entity work.Xor2(Xor2) port map (In0, In1, s0); 	
	Xor_S0_Cin: entity work.Xor2(Xor2) port map (s0, Cin, Sum);
	And_In0_In1: entity work.And2(And2) port map (In0, In1, s1);
	And_S0_Cin: entity work.And2(And2) port map (s0, Cin, s2);
	Or_S1_S2: entity work.Or2(Or2) port map (S1, S2, Cout); 
	-- this is eqivelant to => cout & Sum <= in0 + in1 + Cin. 
	
end architecture structural_One_Bit_Adder;