import {useEffect, useState} from "react";
import {DockingSensor} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

export const useDockingSensor = () => {
    const [dockingSensor, setDockingSensor] = useState<DockingSensor>({})
    const dockingSensorStream = useWS<string>(() => {
            console.log({
                message: "DockingSensor Stream closed",
            })
        }, () => {
            console.log({
                message: "DockingSensor Stream connected",
            })
        },
        (e) => {
            setDockingSensor(JSON.parse(e))
        })
    useEffect(() => {
        dockingSensorStream.start("/api/openmower/subscribe/dockingSensor",)
        return () => {
            dockingSensorStream.stop()
        }
    }, []);
    return dockingSensor;
};
