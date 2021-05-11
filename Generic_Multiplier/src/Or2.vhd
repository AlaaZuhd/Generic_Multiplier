-----------------------------------------------------------
-- implemnation of the OR gate with two inputs. 

-- Defining needed Libraries
library ieee;
use ieee.std_logic_1164.all; 

-- defining the entity of the 2-inputs OR
entity Or2 is
	port(In0, In1: in std_logic;
		res: out std_logic);
end entity Or2;

-- defining and building the architecture of the 2-inputs OR
architecture Or2 of Or2	is
begin	
	 res <= In0 or In1; 
end architecture Or2;