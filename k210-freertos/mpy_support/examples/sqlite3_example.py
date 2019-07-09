import machine, os, usqlite3
from upysh import *

# If the database file is on SD Card, mount it
sdcard = os.VfsSDCard()
os.mount(sdcard, '/sd')

# Enable or disable sqlite3 debug information
usqlite3.debug(True)
#usqlite3.debug(False)

# Open the database
#conn = usqlite3.connect('/flash/chinook.db')
conn = usqlite3.connect('/sd/chinook.db')

curr = conn.cursor()


# === Get the names of all tables in the database ===
curr.execute("SELECT * FROM sqlite_master WHERE type='table'")
# Get column names
curr.description()
'''
('type', 'name', 'tbl_name', 'rootpage', 'sql')
'''
# Fetch all data
tables = curr.fetchall()
# Print data
for row in tables:
    table = tables[row]
    print("Name: {}\r\n SQL:\r\n{}\r\n".format(table[1], table[4]))
'''
Name: albums
 SQL:
CREATE TABLE "albums"
(
    [AlbumId] INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    [Title] NVARCHAR(160)  NOT NULL,
    [ArtistId] INTEGER  NOT NULL,
    FOREIGN KEY ([ArtistId]) REFERENCES "artists" ([ArtistId]) 
		ON DELETE NO ACTION ON UPDATE NO ACTION
)

Name: sqlite_sequence
 SQL:
CREATE TABLE sqlite_sequence(name,seq)

Name: artists
 SQL:
CREATE TABLE "artists"
(
    [ArtistId] INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    [Name] NVARCHAR(120)
)

Name: customers
 SQL:
CREATE TABLE "customers"
(
    [CustomerId] INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    [FirstName] NVARCHAR(40)  NOT NULL,
    [LastName] NVARCHAR(20)  NOT NULL,
    [Company] NVARCHAR(80),
    [Address] NVARCHAR(70),
    [City] NVARCHAR(40),
    [State] NVARCHAR(40),
    [Country] NVARCHAR(40),
    [PostalCode] NVARCHAR(10),
    [Phone] NVARCHAR(24),
    [Fax] NVARCHAR(24),
    [Email] NVARCHAR(60)  NOT NULL,
    [SupportRepId] INTEGER,
    FOREIGN KEY ([SupportRepId]) REFERENCES "employees" ([EmployeeId]) 
		ON DELETE NO ACTION ON UPDATE NO ACTION
)

etc...
'''

# === Get number of rows int table 'tracks' ===
curr.execute("select count(*) from tracks")
curr.fetchone()
'''
('3503',)
'''

# === Select all tracks with name starting with 'Bad' ===
sql = """SELECT
 trackid,
 name,
 composer,
 unitprice
FROM
 tracks
WHERE name LIKE 'Bad%'
ORDER BY TrackID;"""

curr.execute(sql)
tracks = curr.fetchall()

for n in tracks:
    track = tracks[n]
    print("Id: {} Name: {}".format(track[0], track[1]))

'''
True
Id: 18 Name: Bad Boy Boogie
Id: 113 Name: Bad Boy
Id: 678 Name: Bad Moon Rising
Id: 769 Name: Bad Attitude
Id: 892 Name: Badge
Id: 1053 Name: Bad, Bad Leroy Brown
Id: 1164 Name: Bad Obsession
Id: 1171 Name: Bad Apples
Id: 1868 Name: Bad Seed
Id: 3009 Name: Bad
'''

# === Select distinct city names fro 'customer' table ===
sql = """SELECT DISTINCT
 city,
 country
FROM
 customers
WHERE city LIKE 'S%' OR city LIKE 'B%'
ORDER BY city DESC;"""

curr.execute(sql)
cities = curr.fetchall()

for n in cities:
    city = cities[n]
    print("Customer City: {}, {}".format(city[0], city[1]))

'''
Customer City: São Paulo, Brazil
Customer City: São José dos Campos, Brazil
Customer City: Stuttgart, Germany
Customer City: Stockholm, Sweden
Customer City: Sidney, Australia
Customer City: Santiago, Chile
Customer City: Salt Lake City, USA
Customer City: Buenos Aires, Argentina
Customer City: Budapest, Hungary
Customer City: Brussels, Belgium
Customer City: Brasília, Brazil
Customer City: Boston, USA
Customer City: Bordeaux, France
Customer City: Berlin, Germany
Customer City: Bangalore, India
'''


# === Select with Inner join ===
sql = """SELECT
 trackid,
 name,
 composer
FROM
 tracks
INNER JOIN albums ON albums.albumid = tracks.albumid
WHERE name LIKE 'Bad%'
ORDER BY TrackID;
"""

curr.execute(sql)
tracks = curr.fetchall()

for n in tracks:
    track = tracks[n]
    print("Id: {}; Name: {}; Composer: {}".format(track[0], track[1], track[2]))

'''
Id: 18; Name: Bad Boy Boogie; Composer: AC/DC
Id: 113; Name: Bad Boy; Composer: Larry Williams
Id: 678; Name: Bad Moon Rising; Composer: J. C. Fogerty
Id: 769; Name: Bad Attitude; Composer: Richie Blackmore, Ian Gillian, Roger Glover, Jon Lord
Id: 892; Name: Badge; Composer: Clapton/Harrison
Id: 1053; Name: Bad, Bad Leroy Brown; Composer: jim croce
Id: 1164; Name: Bad Obsession; Composer: None
Id: 1171; Name: Bad Apples; Composer: None
Id: 1868; Name: Bad Seed; Composer: Hetfield, Ulrich, Hammett
Id: 3009; Name: Bad; Composer: U2
'''


# === Create a view ===
sql = """CREATE VIEW IF NOT EXISTS v_tracks 
AS 
SELECT
 trackid,
 tracks.name,
 albums.Title AS album,
 media_types.Name AS media,
 genres.Name AS genres
FROM
 tracks
INNER JOIN albums ON albums.AlbumId = tracks.AlbumId
INNER JOIN media_types ON media_types.MediaTypeId = tracks.MediaTypeId
INNER JOIN genres ON genres.GenreId = tracks.GenreId;"""

curr.execute(sql)

# === Select records from the view, limit result to 12 ===
curr.execute("SELECT * FROM v_tracks WHERE (trackid % 100) = 0 order by name LIMIT 12")

tracks = curr.fetchall()

for n in tracks:
    track = tracks[n]
    print("   Id: {}\n Name: {}\n Type: {}\nGenre: {}\n".format(track[0], track[1], track[3], track[4]))

'''
   Id: 400
 Name: Alice
 Type: MPEG audio file
Genre: Latin

   Id: 2500
 Name: Ava Adore
 Type: MPEG audio file
Genre: Alternative & Punk

   Id: 2000
 Name: Breed
 Type: MPEG audio file
Genre: Rock

   Id: 2100
 Name: Crazy Train
 Type: MPEG audio file
Genre: Metal

   Id: 1600
 Name: D'Yer Mak'er
 Type: MPEG audio file
Genre: Rock

   Id: 1200
 Name: Dark Side Of The Cog
 Type: MPEG audio file
Genre: Jazz

   Id: 1700
 Name: Dezesseis
 Type: MPEG audio file
Genre: Latin

   Id: 2300
 Name: E-Bow The Letter
 Type: MPEG audio file
Genre: Rock

   Id: 2900
 Name: Exposé
 Type: Protected MPEG-4 video file
Genre: Drama

   Id: 800
 Name: Fire In The Basement
 Type: MPEG audio file
Genre: Rock

   Id: 3200
 Name: Gay Witch Hunt
 Type: Protected MPEG-4 video file
Genre: TV Shows

   Id: 1100
 Name: Ghandi (Live)
 Type: MPEG audio file
Genre: Latin
'''

# === Drop the view ===

curr.execute("DROP VIEW v_tracks")



# === Close the database ===

conn.close()

