-----------------------------------------------------------
-- implemnation of the XOR gate with two inputs. 

-- Defining needed Libraries
library ieee;
use ieee.std_logic_1164.all; 

-- defining the entity of the 2-inputs XOR
entity Xor2 is
	port(In0, In1: in std_logic;
		res: out std_logic);
end entity Xor2;

-- defining and building the architecture of the 2-inputs XOR
architecture Xor2 of Xor2 is
begin	
	 res <= In0 xor In1; 
end architecture Xor2;