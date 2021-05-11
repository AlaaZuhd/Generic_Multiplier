-----------------------------------------------------------
-- implemnation of the And gate with two inputs. 

-- Defining needed Libraries
library ieee;
use ieee.std_logic_1164.all; 

-- defining the entity of the 2-inputs And
entity And2 is
	port(In0, In1: in std_logic;
		res: out std_logic);
end entity And2;					

-- defining and building the architecture of the 2-inputs And
architecture And2 of And2 is
begin	
	res <= In0 and In1;
end architecture And2;