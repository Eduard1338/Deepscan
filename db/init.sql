CREATE TABLE IF NOT EXISTS detections (
    id SERIAL PRIMARY KEY,
    mission_id INT,
    timestamp TIMESTAMPTZ DEFAULT NOW(),
    world_x DOUBLE PRECISION,
    world_y DOUBLE PRECISION,
    world_z DOUBLE PRECISION,
    radius DOUBLE PRECISION,
    width DOUBLE PRECISION,
    length DOUBLE PRECISION,
    height DOUBLE PRECISION,
    point_count INT
);

CREATE TABLE IF NOT EXISTS missions (
    id SERIAL PRIMARY KEY,
    start_time TIMESTAMPTZ DEFAULT NOW(),
    end_time TIMESTAMPTZ,
    map_file TEXT
);

CREATE TABLE IF NOT EXISTS logs (
    id SERIAL PRIMARY KEY,
    timestamp TIMESTAMPTZ DEFAULT NOW(),
    level TEXT,
    source TEXT,
    message TEXT
);
