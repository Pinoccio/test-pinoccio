-- from http://www.gammon.com.au/forum/?id=11635
--

-- Do MD5 sumcheck of Arduino bootloader .hex file
-- Nick Gammon
-- 5 May 2012

local select, pairs, tonumber, tostring, unpack, xpcall, assert, getmetatable, setmetatable, type, pcall =
	   select, pairs, tonumber, tostring, unpack, xpcall, assert, getmetatable, setmetatable, type, pcall
local sort, format, byte, char, min, max =
	table.sort, string.format, string.byte, string.char, math.min, math.max


loader = nil
adder = 0

-- given a start address, deduce where the bootloader ends
end_addresses = {
  [0x0000] = 0x40000,
  [0x1C00] = 0x2000,
  [0x1D00] = 0x2000,
  [0x1E00] = 0x2000,
  [0x3800] = 0x4000,
  [0x3E00] = 0x4000,
  [0x7000] = 0x8000,
  [0x7800] = 0x8000,
  [0x7E00] = 0x8000,
  [0xF800] = 0x10000,
  [0x1F000] = 0x20000,
  [0x1FC00] = 0x20000,
  [0x3E000] = 0x40000,
  }

function getlines(str)
  local pos = 0

  -- the for loop calls this for every iteration
  -- returning nil terminates the loop
  local function iterator (s)

    if not pos then
      return nil
    end -- end of string, exit loop

    local oldpos = pos + 1  -- step past previous newline
    pos = string.find (s, "\n", oldpos) -- find next newline

    if not pos then  -- no more newlines, return rest of string
      return string.sub (s, oldpos)
    end -- no newline

    return string.sub (s, oldpos, pos - 1)

  end -- iterator

  return iterator, str
end -- getlines

function tohex(s, upper)
	if type(s) == 'number' then
		return format(upper and '%08.8X' or '%08.8x', s)
	end
	if upper then
		return (s:gsub('.', function(c)
		  return format('%02X', byte(c))
		end))
	else
		return (s:gsub('.', function(c)
		  return format('%02x', byte(c))
		end))
	end
end

function fromhex(s)
	return (s:gsub('..', function(cc)
	  return char(tonumber(cc, 16))
	end))
end

function process (size, address, rectype, data)

  size = tonumber (size, 16)
  address = tonumber (address, 16) + adder
  rectype = tonumber (rectype)
  local binarydata = fromhex (data)

  if rectype == 2 then  -- Extended Segment Address Record
    adder = tonumber (data, 16) * 16  -- high order address byte
  elseif rectype == 0 then -- data record
    if loader == nil then
      start_address = address
      end_address = end_addresses [address]
      if end_address == nil then
       --  ColourNote ("red", "", "Don't know end address for " .. bit.tostring (address, 16))
        print ("Don't know end address for " .. address)
        print ("Please add to table: end_addresses")
        error "Cannot continue"
      end -- if end address not found

      -- work out loader length
      length = end_address - address
      -- pre-fill with 0xFF in case not every byte supplied
      loader = string.rep ("\255", length)
      print (string.format ("// Loader start: %X, length: %i", address, length))
    end -- no loader yet

    -- insert data over where the 0xFF was
    if address >= start_address and (address + size) <= end_address then
      loader = loader:sub (1, address - start_address) ..
               binarydata ..
               loader:sub (address - start_address + size + 1, length)
    else
      -- ColourNote ("red", "", "Address " .. bit.tostring (address, 16) .. " out of expected range.")
      print ("Address " .. address .. " out of expected range.")
    end  -- if in range
  end -- if

end -- function process

print (string.rep ("-", 60))

-- get bootloader file
filename = "/Users/eric/Desktop/Scout.cpp.hex"

-- none chosen, give up
if not filename then return end

-- show file
-- local fn = string.match (filename, "\\([^\\]+)$")
local fn = filename
print ("// File = ", fn)

-- process each line
for line in io.lines (filename) do
  size, address, rectype, data = string.match (line, "^:(%x%x)(%x%x%x%x)(%x%x)(%x+)%s*$")
  if size then
    process (size, address, rectype, data:sub (1, -3))
  else
    ColourNote ("red", "", "Discarded line: " .. line)
  end -- if
end -- for loop

--print (tohex (loader))

-- sumcheck it
-- Tell ("// MD5 sum = ")
-- md5sum = tohex (utils.md5 (loader))
-- print ((string.gsub (md5sum, "(%x%x)", "%1 ")))

print ""

--     show bootloader in hex

-- convert into C array
print (string.format ("byte PROGMEM %s [] = {",
       string.gsub (fn, "[^%a%d]", "_")))
for i = 1, #loader do
  -- Tell (string.format ("0x%02X, ", loader:sub (i, i):byte ()))
  if (i - 1) % 16 == 15 then print "" end
end -- of for each byte
print (string.format ("}; // end of %s", string.gsub (fn, "[^%a%d]", "_")))
