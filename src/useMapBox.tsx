import { useState, useEffect, useRef, MutableRefObject } from 'react';
import mapboxgl from 'mapbox-gl';
import { BaloonData } from '../models/BaloonData';
// @ts-ignore
import * as turf from '@turf/turf';
import 'mapbox-gl/dist/mapbox-gl.css';
import { DataPacket } from '../interfaces/DataPacket';

// Use new valid mapbox access token
mapboxgl.accessToken =
  'pk.eyJ1IjoicmtlZyIsImEiOiJjbWJwa3Q4dmwwNjZyMmtxNG4zZHpub21xIn0.5IQ061SZLu_F4_ptyu-Sbg';
// rkeg "pk.eyJ1IjoicmtlZyIsImEiOiJjbWJwa3Q4dmwwNjZyMmtxNG4zZHpub21xIn0.5IQ061SZLu_F4_ptyu-Sbg";
// EVANS "pk.eyJ1IjoicGFyZHVobmUiLCJhIjoiY2xsemhod2NlMml6eDNtczVua2JxY2p5ZyJ9.qqm-fK1ySqzrvwCFwK9EIQ";
// JASONS "pk.eyJ1IjoiamFzb25rcnVlZ2VyIiwiYSI6ImNsbHF4d2ltZzBnZDgzY21sY29xODNsMHUifQ.9RxwOQueLDzOUl-KFXHOAg";

interface useMapBoxProps {
  lat: number;
  lng: number;
  zoom?: number;
  data?: BaloonData;
  replayControl?: {
    replaySpeed: number;
    replayTrigger?: number;
    stopReplayTrigger?: number;
  };
  onReplayEnd?: () => void;
}

/*
 * Filters out packets with invalid latitude and longitude
 * vales and also removes any duplicate packets.
 *
 * @param {BaloonData} data - BaloonData instance containing a packets array.
 * @returns {DataPacket[]} - Array of DataPacket objects that were not filtered out.
 */
const getValidPackets = (data: BaloonData) => {
  const validPackets = data
    .getPackets()
    .filter(
      (packet) =>
        packet.latitude &&
        packet.longitude &&
        !isNaN(packet.latitude) &&
        !isNaN(packet.longitude)
    );

  const uniquePackets = validPackets.filter((packet, index) => {
    if (index === 0) return true;
    const prevPacket = validPackets[index - 1];
    return (
      packet.latitude !== prevPacket.latitude ||
      packet.longitude !== prevPacket.longitude
    );
  });

  return uniquePackets;
};

/*
 * Clear all markers from the map
 *
 * @param {MutableObject<mapboxgl.Marker[]>} ref - Reference to an array of markers.
 */
const clearMarkers = (ref: MutableRefObject<mapboxgl.Marker[]>) => {
  ref.current.forEach((marker) => {
    marker.remove();
  });
};

/*
 * Creates a dot marker html element with a specified color.
 *
 * @param {string} color - The color of the dot marker.
 * @returns {HTMLElement} - The created dot marker element.
 */
const createDotMarker = (color: string) => {
  const element = document.createElement('div');
  element.style.width = '10px';
  element.style.height = '10px';
  element.style.borderRadius = '50%';
  element.style.backgroundColor = color;
  element.style.border = '1px solid black';
  return element;
};

/*
 * Creates a burst marker html element with a specified color.
 *
 * @param {string} clor - The color of the burst marker.
 * @returns {HTMLElement} - The created burst marker element.
 */
const createBurstMarker = (color: string) => {
  const element = document.createElement('div');
  element.innerHTML = '✸'; // burst symbol
  element.style.fontSize = '20px';
  element.style.color = color;
  element.style.textShadow =
    '-2px -2px 0 #000, 2px -2px 0 #000, -2px 2px 0 #000, 2px 2px 0 #000';
  return element;
};

/*
 * Creates a star marker html element with a specified color.
 *
 * @param {string} color - The color of the star marker.
 * @returns {HTMLElement} - The created star marker element.
 */
const createStarMarker = (color: string) => {
  const element = document.createElement('div');
  element.innerHTML = '★'; // star symbol
  element.style.fontSize = '15px';
  element.style.color = color;
  element.style.textShadow =
    '-1px -1px 0 #000, 1px -1px 0 #000, -1px 1px 0 #000, 1px 1px 0 #000';
  return element;
};

/*
 * Creates a popup for a given data packet's marker.
 *
 * @param {DataPacket} packet - The data packet to create a popup for.
 * @returns {mapboxgl.Popup} - The created popup instance.
 */
const createPopup = (packet: DataPacket) => {
  const popup = new mapboxgl.Popup({
    offset: 25,
    closeButton: true,
    closeOnClick: false,
    closeOnMove: false,
    className: 'packet-popup custom-popup',
  }).setHTML(`
  <div class="popup-container">
    <h4 class="popup-header">Packet @ ${
      packet.time ? new Date(packet.time).toLocaleString() : 'N/A'
    }</h4>
    <div class="popup-grid">
      <p class="popup-data"><span class="label">Spd:</span> <span class="value">${packet.speed.toFixed(
        2
      )} MPH</span></p>
      <p class="popup-data"><span class="label">Head:</span> <span class="value">${packet.heading.toFixed(
        0
      )}°</span></p>
      <p class="popup-data"><span class="label">Alt:</span> <span class="value">${packet.altitude.toFixed(
        2
      )} ft.</span></p>
      <p class="popup-data"><span class="label">Asc:</span> <span class="value">${packet.verticalRate.toFixed(
        0
      )} ft./min.</span></p>
      <p class="popup-data"><span class="label">Lat:</span> <span class="value">${packet.latitude.toFixed(
        2
      )}°</span></p>
      <p class="popup-data"><span class="label">Long:</span> <span class="value">${packet.longitude.toFixed(
        2
      )}°</span></p>
      ${packet.burst ? '<p class="popup-burst">BURST</p>' : ''}
    </div>
  </div>
  `);
  return popup;
};

/*
 * Creates a marker element for a given data packet.
 * If the packet is a burst, it creates a burst marker, otherwise a dot marker.
 *
 * @param {DataPacket} packet - The data packet to create a marker for.
 * @returns {HTMLElement} - The created marker element.
 */
const markerElement = (packet: DataPacket) => {
  return packet.burst ? createBurstMarker('#f00') : createDotMarker('#f00');
};

// Indianapolis IN coords:
// 39.77175159522747, -86.14992561849408
export const useMapBox = ({
  lat,
  lng,
  zoom = 10,
  data,
  replayControl,
  onReplayEnd,
}: useMapBoxProps) => {
  const mapContainer = useRef<HTMLDivElement>(null);
  const map = useRef<mapboxgl.Map | null>(null);
  const markersRef = useRef<mapboxgl.Marker[]>([]);
  const replayMarkersRef = useRef<mapboxgl.Marker[]>([]);
  // replay reference to persist state across re-renders
  const replayStateRef = useRef<{
    currentIndex: number;
    previousStarMarker: mapboxgl.Marker | null;
    timerId: NodeJS.Timeout | null;
    cancelled: boolean;
    switching3D?: boolean;
    packets: DataPacket[];
  }>({
    currentIndex: 0,
    previousStarMarker: null,
    timerId: null,
    cancelled: false,
    switching3D: false,
    packets: [],
  });
  const [mapLoaded, setMapLoaded] = useState<boolean>(false);
  const [showAllMarkers, setShowAllMarkers] = useState<boolean>(false);
  const [initialViewSet, setInitialViewSet] = useState<boolean>(false);
  const [isReplaying, setIsReplaying] = useState<boolean>(false);
  const [isStandardStyle, setIsStandardStyle] = useState<boolean>(true);
  const [is3DMode, setIs3DMode] = useState<boolean>(false);

  /*
   * Initializes the Mapbox map instance with a given style, center, and zoom level.
   * Adds navigation controls and sets up event listeners for error handling on map and style loading.
   * Finds the midpoint of the given mission data and flies to that location on initialization.
   *
   * @effect Initializes map, sets up controls, and flies to midpoint if available.
   * @triggers Runs on component mount
   * @cleanup None
   */
  useEffect(() => {
    if (map.current) return; // initialize map only once

    map.current = new mapboxgl.Map({
      container: mapContainer.current as HTMLElement,
      // Default "mapbox://styles/mapbox/streets-v12"
      // Jasons "mapbox://styles/jasonkrueger/ckeikjwuz0f291aqs36za0dgt"
      // Evans "mapbox://styles/parduhne/cllv9mf8m010401p7h6z32h0c"
      // rkeg "mapbox://styles/rkeg/cmbqt89nd00rv01s6dxbmhvli"
      style: 'mapbox://styles/rkeg/cmbqt89nd00rv01s6dxbmhvli',
      center: [lng, lat],
      zoom: zoom,
      pitch: 0,
      bearing: 0,
      dragRotate: false,
      touchPitch: false,
    });

    map.current.addControl(new mapboxgl.NavigationControl(), 'bottom-right');

    map.current.on('error', (e) => {
      console.error('Map error:', e);
    });

    map.current.on('style.load', () => {
      setMapLoaded(true);
    });

    map.current.on('error', (e) => {
      console.error('Map error:', e);
    });

    if (data) {
      const midpoint = data.getMidpoint();
      if (midpoint) {
        map.current.flyTo({
          center: midpoint as mapboxgl.LngLatLike,
          essential: true,
        });
      }
    }
  }, []);

  // Get current map style for toggling html in Map component
  let currentMapStyle;

  if (isStandardStyle) {
    currentMapStyle = 'street';
  } else {
    currentMapStyle = 'satellite';
  }

  const debugAllMapLayers = () => {
  if (!map.current) return;
  
  const style = map.current.getStyle();
  
  console.log('=== ALL LAYERS ===');
  style.layers?.forEach((layer, index) => {
    console.log(`${index}: ${layer.id} (${layer.type})`);
  });
  
  console.log('=== ALL SOURCES ===');
  Object.keys(style.sources || {}).forEach(sourceId => {
    console.log(`Source: ${sourceId}`, style.sources[sourceId]);
  });
  
  console.log('=== LINE LAYERS ONLY ===');
  const lineLayers = style.layers?.filter(layer => layer.type === 'line');
  lineLayers?.forEach(layer => {
    console.log(`Line layer: ${layer.id}, source: ${layer.source}`);
  });
};

  /*
   * Toggles the map style between standard and satellite views.
   * Also puts the routes back up after the style change.
   */
  const toggleMapStyle = () => {
    if (!map.current) return;

    // Satellite: mapbox://styles/rkeg/cmbzhm2bx00no01rx6coe8mjo
    // Street: mapbox://styles/rkeg/cmbqt89nd00rv01s6dxbmhvli
    const newStyle = isStandardStyle
      ? 'mapbox://styles/rkeg/cmbzhm2bx00no01rx6coe8mjo' // Satellite style
      : 'mapbox://styles/rkeg/cmbqt89nd00rv01s6dxbmhvli'; // Streets style

    const onStyleLoad = () => {
      setMapLoaded(true);
      if (!isReplaying) {
        setupRouteAndPoints(is3DMode);
      }
      map.current?.off('style.load', onStyleLoad);
    };

    map.current.on('style.load', onStyleLoad);
    map.current.setStyle(newStyle);
    setIsStandardStyle(!isStandardStyle);
    setMapLoaded(false);
  };

  const addTerrain = () => {
    if (!map.current) return;

    try {
      map.current.addSource('mapbox-dem', {
        type: 'raster-dem',
        url: 'mapbox://mapbox.mapbox-terrain-dem-v1',
        tileSize: 512,
        maxzoom: 14,
      });
      map.current.setTerrain({ source: 'mapbox-dem', exaggeration: 1.5 });
    } catch (error) {
      console.error('Error adding terrain:', error);
    }
  };

  const removeTerrain = () => {
    if (!map.current) return;

    try {
      map.current.setTerrain(null);

      if (map.current.getSource('mapbox-dem')) {
        map.current.removeSource('mapbox-dem');
      }
    } catch (error) {
      console.error('Error removing terrain:', error);
    }
  };

  /*
   * Sets up the route layer on the map with the given data.
   * It removes any existing route layer and source, then adds a new source
   */
  const setupRouteAndPoints = (is3D: boolean) => {
    if (!map.current || !data || !mapLoaded) return;

    console.log('=== SETTING UP ROUTE AND POINTS ===');

    if (!is3D) {
      console.log('Setting up 2D route');
      removeTerrain();
      teardown2DRoute();
      setup2DRoute();
    } else if (is3D) {
      console.log('Setting up 3D route');
      addTerrain();
      teardown3DRoute();
      setup3DRoute();
    }
  };

  // Tears down the route layer and source from the map.
 const teardown2DRoute = () => {
  if (!map.current) return;

  try {
    console.log('Attempting to teardown 2D route');
    
    if (map.current.getLayer('2d-route')) {
      console.log('Removing 2d-route layer');
      map.current.removeLayer('2d-route');
    } else {
      console.log('2d-route layer not found');
    }

    if (map.current.getSource('2d-route')) {
      console.log('Removing 2d-route source');
      map.current.removeSource('2d-route');
    } else {
      console.log('2d-route source not found');
    }
  } catch (error) {
    console.error('Error tearing down 2D route:', error);
  }
};

  const teardown3DRoute = () => {
    if (!map.current) return;

    try {
      if (map.current.getLayer('3d-route')) {
        map.current.removeLayer('3d-route');
      }

      if (map.current.getSource('3d-route')) {
        map.current.removeSource('3d-route');
      }
    } catch (error) {
      console.error('Error tearing down 3D route:', error);
    }
  };

  // Sets up the route layer and markers for the packets in the data.
  const setup2DRoute = () => {
    if (!map.current || !data || !mapLoaded) return;

    console.log('=== SETTING UP 2D ROUTE ===');
    console.trace('Called from:'); // This will show the call stack

    try {
      const packets = getValidPackets(data);

      // Gives each packet attributes to be used in the GeoJSON LineString
      packets.map((packet) => ({
        type: 'Feature' as const,
        properties: {
          burst: packet.burst,
          altitude: packet.altitude,
        },
        geometry: {
          type: 'Point' as const,
          coordinates: [packet.longitude, packet.latitude],
        },
      }));

      // Add a source for the packet positions
      map.current.addSource('2d-route', {
        type: 'geojson',
        data: {
          type: 'Feature',
          properties: {},
          geometry: {
            type: 'LineString',
            coordinates: data.getPositionData(),
          },
        },
      });

      // Adds a layer for the line detailing the packets' route
      map.current.addLayer({
        id: '2d-route',
        type: 'line',
        source: '2d-route',
        layout: {
          'line-join': 'round',
          'line-cap': 'round',
        },
        paint: {
          'line-color': '#f00',
          'line-width': 2.5,
        },
      });
    } catch (error) {
      console.error('Error setting up 2D route:', error);
    }
  };

  const setup3DRoute = () => {
    if (!map.current || !data || !mapLoaded) return;

    console.log('=== SETTING UP 3D ROUTE ===');
    console.trace('Called from:'); // This will show the call stack

    try {
      const packets = getValidPackets(data);

      // Add the GeoJSON source with the coordinates and their elevations
      map.current.addSource('3d-route', {
        type: 'geojson',
        lineMetrics: true,
        data: {
          type: 'Feature',
          properties: {
            elevation: packets.map((packet) => packet.altitude * 0.3048), // Convert feet to meters
          },
          geometry: {
            coordinates: packets.map((packet) => [
              packet.longitude,
              packet.latitude,
            ]),
            type: 'LineString',
          },
        },
      });

      // Add the elevated line layer
      map.current.addLayer({
        id: '3d-route',
        type: 'line',
        source: '3d-route',
        layout: {
          'line-z-offset': [
            'at-interpolated',
            ['*', ['line-progress'], ['-', ['length', ['get', 'elevation']], 1]],
            ['get', 'elevation'],
          ],
          'line-elevation-reference': 'sea',
        },
        paint: {
          'line-emissive-strength': 1.0,
          'line-width': 8,
          'line-color': '#f00',
        },
      });
    } catch (error) {
      console.error('Error setting up 3D route:', error);
    }
  };

  // Toggle enable 3D map and 2D display and features
  const toggle3DMode = () => {
    if (!map.current) return;

    console.log('=== BEFORE 3D TOGGLE ===');
    //debugAllMapLayers();

    const new3DMode = !is3DMode;

    setIs3DMode(new3DMode);

    console.log(`new3DMode: ${new3DMode}`);
    console.log(`isReplaying: ${isReplaying}`);
    if (new3DMode && isReplaying) {
      cancelReplay(true);
    }

    if (new3DMode) {
      console.log('Switching to 3D mode');
      teardown2DRoute();
      teardown3DRoute();
    } else {
      console.log('Switching to 2D mode');
      teardown3DRoute();
      teardown2DRoute();
    }

    console.log('=== AFTER CLEANUP ===');
    //debugAllMapLayers()

    if (new3DMode) {
      addTerrain();
      setup3DRoute();
      //setupRouteAndPoints(new3DMode);
      map.current.dragRotate.enable();
      map.current.touchPitch.enable();
      map.current.easeTo({
        pitch: 45,
        duration: 1000,
      });
    } else {
      //setupRouteAndPoints(new3DMode);
      removeTerrain();
      setup2DRoute();
      map.current.dragRotate.disable();
      map.current.touchPitch.disable();
      map.current.easeTo({
        pitch: 0,
        duration: 1000,
      });
    }

    console.log('=== AFTER SETUP ===');
    //debugAllMapLayers();
    
  };

  const cancelReplay = (switching3D = false) => {
    const state = replayStateRef.current;
    state.cancelled = true;
    state.switching3D = switching3D;

    console.log(`is 3DMode: ${is3DMode}`);
    console.log(`switching3D: ${switching3D}`);
    
    if (state.timerId) {
      clearTimeout(state.timerId);
      state.timerId = null;
    }
    
    if (state.previousStarMarker) {
      state.previousStarMarker.remove();
    }
    
    if (replayMarkersRef.current.length > 0) {
      clearMarkers(replayMarkersRef);
      replayMarkersRef.current = [];
    }

    setIsReplaying(false);
    
    if (!switching3D) {
      setupRouteAndPoints(is3DMode);
    }

    if (onReplayEnd) {
      onReplayEnd();
    }

  };

  /*
   * Sets up the initial view of the map by adding a route layer and markers for
   * the packets if the map and data are available.
   * Also fits the map bounds to the route and sets the initial view flag.
   *
   * @effect Sets up the initial view of the map with route and markers.
   * @triggers Runs on data, map, mapLoaded, and initialViewSet changes
   * @cleanup None
   */
  useEffect(() => {
    if (!data || !map.current || !mapLoaded || initialViewSet) return;

    try {
      setupRouteAndPoints(is3DMode);

      // Gets the bounds of the route and fits the map to it
      const bounds = turf.bbox(turf.lineString(data.getPositionData())) as [
        number,
        number,
        number,
        number
      ];
      map.current.fitBounds(bounds, { padding: 100 });

      // Once done with initial map setup, set the initial view flag
      setInitialViewSet(true);
    } catch (error) {
      console.error('Error setting initial view:', error);
    }
  }, [data, map.current, mapLoaded, initialViewSet]);

  /*
   * Clears existing markers if any, and adds markers for each packet in the data.
   * If it's not replaying, it adds a location marker for the last packet.
   * If showAllMarkers is true, it adds dot or burst markers for all packets,
   * not including the last one if not replaying.
   *
   * @effect Clears any existing markers and adds new ones based on constraints.
   * @triggers Runs on data, map, mapLoaded, showAllMarkers, initialViewSet, and isReplaying changes
   * @cleanup Clears markers or unmount or when not showing all markers,
   * except for the last packet if not replaying.
   */
  useEffect(() => {
    if (!data || !map.current || !mapLoaded || !initialViewSet) return;

    try {
      const packets = getValidPackets(data);

      clearMarkers(markersRef);

      // If not replaying, add a location marker for the last packet and add to reference
      if (!isReplaying) {
        const lastPacket = data.getLatestPacket();

        const lastPacketMarker = new mapboxgl.Marker({
          color: '#f00',
          draggable: false,
          scale: 0.8,
        })
          .setLngLat([lastPacket.longitude, lastPacket.latitude])
          .setPopup(createPopup(lastPacket))
          .addTo(map.current as mapboxgl.Map);

        markersRef.current.push(lastPacketMarker);
      }

      /* If showing all markers, add dot or burst markers for each packet
       * including the last one if replaying, and add them to reference.
       */
      if (showAllMarkers) {
        const rangeEnd = isReplaying ? packets.length : -1;
        packets.slice(0, rangeEnd).forEach((packet) => {
          const marker = new mapboxgl.Marker({
            element: markerElement(packet),
            draggable: false,
          })
            .setLngLat([packet.longitude, packet.latitude])
            .setPopup(createPopup(packet))
            .addTo(map.current as mapboxgl.Map);

          markersRef.current.push(marker);
        });
      }
    } catch (error) {
      console.error('Error updating map:', error);
    }
  }, [
    map.current,
    data,
    mapLoaded,
    showAllMarkers,
    initialViewSet,
    isReplaying,
    is3DMode,
  ]);

  /*
   * Replays the balloon data by iterating through the packets and creating
   * a star marker for the current packet in the iteration, and making
   * dot or burst markers for the previously iterated packets. Clears
   * markers before starting, when replay is stopped, or when replay finishes.
   *
   * @effect Makes markers for each packet in the replay iteration, with the current
   * packet being a star marker and the previous packets being dot or burst markers.
   * @triggers Runs on replayControl changes, map, mapLoaded, and data changes
   * @cleanup Clears markers on unmount or when replay is stopped or finished.
   */
  useEffect(() => {
    if (
      !data ||
      !map.current ||
      !mapLoaded ||
      !replayControl ||
      replayControl.replayTrigger === undefined ||
      is3DMode
    )
      return;

    teardown2DRoute();

    const packets = getValidPackets(data);

    replayStateRef.current = {
      currentIndex: 0,
      previousStarMarker: null,
      timerId: null,
      cancelled: false,
      packets,
    };

    const replayStep = () => {
      const state = replayStateRef.current;

      if (state.cancelled || state.currentIndex >= state.packets.length) {
        setIsReplaying(false);
        if (!state.cancelled && onReplayEnd) {
          onReplayEnd();
        }
        return;
      }

      const packet = state.packets[state.currentIndex];
      const speed = replayControl.replaySpeed || 1;
      const baseDelay = 10000;
      const delay = baseDelay / speed;

      /* If the previous star marker exists remove it
       * and make it a burst or dot marker instead
       */
      if (state.previousStarMarker) {
        state.previousStarMarker.remove();

        const prevPacket = state.packets[state.currentIndex - 1];
        const marker = new mapboxgl.Marker({
          element: markerElement(prevPacket),
          draggable: false,
        })
          .setLngLat([prevPacket.longitude, prevPacket.latitude])
          .setPopup(createPopup(prevPacket))
          .addTo(map.current as mapboxgl.Map);

        replayMarkersRef.current.push(marker);
      }

      // Create a new star marker for the current packet
      const replayMarker = new mapboxgl.Marker({
        element: createStarMarker('#FFFF00'),
        draggable: false,
      })
        .setLngLat([packet.longitude, packet.latitude])
        .setPopup(createPopup(packet))
        .addTo(map.current as mapboxgl.Map);

      state.previousStarMarker = replayMarker;
      state.currentIndex++;

      // Schedule next step with current speed
      state.timerId = setTimeout(replayStep, delay);
    };

    setIsReplaying(true);
    clearMarkers(markersRef);
    clearMarkers(replayMarkersRef);
    replayMarkersRef.current = [];

    replayStep();

    // Cleanup function to cancel the replay and clear markers
    return () => {
      const state = replayStateRef.current;
      state.cancelled = true;
    
      if (state.timerId) {
        clearTimeout(state.timerId);
        state.timerId = null;
      }
      
      if (state.previousStarMarker) {
        state.previousStarMarker.remove();
      }
      
      if (replayMarkersRef.current.length > 0) {
        clearMarkers(replayMarkersRef);
        replayMarkersRef.current = [];
      }

      setIsReplaying(false);

      if (map.current && mapLoaded && data && !state.switching3D) {
        console.log(`is3DMode in cleanup: ${is3DMode}`);
        setupRouteAndPoints(is3DMode);
      }
      
      if (onReplayEnd) {
        onReplayEnd();
      }
    };
  }, [replayControl?.replayTrigger, replayControl?.stopReplayTrigger]);

  /*
   * Changes the replay speed based on the selected option from the dropdown
   * especially if the replay is currently running.
   *
   * @effect Changes the replay speed dynamically while replaying.
   * @triggers Runs on replayControl.speed changes and isReplaying state changes.
   * @cleanup None
   */
  useEffect(() => {
    if (!isReplaying || !replayControl) return;

    const state = replayStateRef.current;

    // Clear current timeout and restart with new speed
    if (state.timerId) {
      clearTimeout(state.timerId);

      const speed = replayControl.replaySpeed || 1;
      const baseDelay = 10000;
      const delay = baseDelay / speed;

      // Continue from where we left off with new speed
      const replayStep = () => {
        if (state.cancelled || state.currentIndex >= state.packets.length) {
          setIsReplaying(false);
          if (!state.cancelled && onReplayEnd) {
            onReplayEnd();
          }
          return;
        }

        const packet = state.packets[state.currentIndex];
        const currentSpeed = replayControl.replaySpeed || 1;
        const currentDelay = baseDelay / currentSpeed;

        if (state.previousStarMarker) {
          state.previousStarMarker.remove();

          const prevPacket = state.packets[state.currentIndex - 1];
          const marker = new mapboxgl.Marker({
            element: markerElement(prevPacket),
            draggable: false,
          })
            .setLngLat([prevPacket.longitude, prevPacket.latitude])
            .setPopup(createPopup(prevPacket))
            .addTo(map.current as mapboxgl.Map);

          replayMarkersRef.current.push(marker);
        }

        const replayMarker = new mapboxgl.Marker({
          element: createStarMarker('#FFFF00'),
          draggable: false,
        })
          .setLngLat([packet.longitude, packet.latitude])
          .setPopup(createPopup(packet))
          .addTo(map.current as mapboxgl.Map);

        state.previousStarMarker = replayMarker;
        state.currentIndex++;

        state.timerId = setTimeout(replayStep, currentDelay);
      };

      state.timerId = setTimeout(replayStep, delay);
    }
  }, [replayControl?.replaySpeed, isReplaying]);

  /* Hook returns the map container, showAllMarkers state,
   * setShowAllMarkers function, mapLoaded state, isReplaying state,
   * toggleMapStyle function, and the currentMapStyle variable
   * to be used in map component and the mission home component.
   */
  return {
    mapContainer,
    showAllMarkers,
    setShowAllMarkers,
    mapLoaded,
    isReplaying,
    toggleMapStyle,
    currentMapStyle,
    is3DMode,
    toggle3DMode,
  };
};

